#ifdef ENABLE_HOLO_AUDIO
#include "HoloCodecOpus.hpp"

using namespace holo;
using namespace holo::codec;

HoloCodecOpus::HoloCodecOpus() : IHoloCodec<std::vector<unsigned char>>(),
	isInit_(false),
	audioFormat_(),
	bitRate_(HOLO_AUDIO_DEFAULT_ENCODE_BITRATE),
	audioEncoder_(nullptr),
	audioDecoder_(nullptr)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.opus");
	audioFormat_.frequency = HOLO_AUDIO_DEFAULT_FMT_FREQ;
	audioFormat_.numChannels = HOLO_AUDIO_DEFAULT_FMT_CHAN;
	audioFormat_.depth = HOLO_AUDIO_DEFAULT_FMT_DEPTH;

	rawFrameLength_ = HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE * audioFormat_.numChannels * sizeof(opus_int16);

	LOG4CXX_DEBUG(logger_, "Opus audio encoder object instantiated with default values");
}

HoloCodecOpus::HoloCodecOpus(HoloAudioFormat audioFormat, int bitRate) : IHoloCodec<std::vector<unsigned char>>(),
	isInit_(false),
	audioFormat_(audioFormat),
	bitRate_(bitRate),
	audioEncoder_(nullptr),
	audioDecoder_(nullptr)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.opus");

	rawFrameLength_ = HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE * audioFormat_.numChannels * sizeof(opus_int16);

	LOG4CXX_DEBUG(logger_, "Opus audio encoder object instantiated with specific values");
}

HoloCodecOpus::~HoloCodecOpus()
{
	deinit();
}

bool HoloCodecOpus::init(CODEC_MODE codecMode)
{
	if (!isInit())
	{
		bool isEncoderInit = false;
		bool isDecoderInit = false;

		codecMode_ = codecMode;
		int errCode = OPUS_OK;

		if (codecMode_ == CODEC_MODE_BOTH || codecMode_ == CODEC_MODE_ENCODER)
		{
			LOG4CXX_INFO(logger_, "Initializing Opus audio encoder...");
			LOG4CXX_INFO(logger_, "Audio frequency: " << audioFormat_.frequency);
			LOG4CXX_INFO(logger_, "Audio number of channels: " << audioFormat_.numChannels);
			LOG4CXX_INFO(logger_, "Audio bitrate: " << bitRate_);

			audioEncoder_ = opus_encoder_create(audioFormat_.frequency, audioFormat_.numChannels, OPUS_APPLICATION_VOIP, &errCode);
			if (errCode != OPUS_OK)
			{
				LOG4CXX_ERROR(logger_, "Could not initialize Opus encoder: " << opus_strerror(errCode));
				deinit();
				return false;
			}

			errCode = opus_encoder_ctl(audioEncoder_, OPUS_SET_BITRATE(bitRate_));
			if (errCode != OPUS_OK)
			{
				LOG4CXX_ERROR(logger_, "Could not set Opus encoder bitrate: " << opus_strerror(errCode));
				deinit();
				return false;
			}

			errCode = opus_encoder_ctl(audioEncoder_, OPUS_SET_SIGNAL(HOLO_AUDIO_DEFAULT_ENCODE_SIGNAL));
			if (errCode != OPUS_OK)
			{
				LOG4CXX_ERROR(logger_, "Could not set Opus encoder signal type: " << opus_strerror(errCode));
				deinit();
				return false;
			}

			errCode = opus_encoder_ctl(audioEncoder_, OPUS_SET_BANDWIDTH(HOLO_AUDIO_DEFAULT_ENCODE_BANDWIDTH));
			if (errCode != OPUS_OK)
			{
				LOG4CXX_ERROR(logger_, "Could not set Opus encoder bandwidth: " << opus_strerror(errCode));
				deinit();
				return false;
			}

			LOG4CXX_INFO(logger_, "Initialized Opus audio encoder");

			isEncoderInit = true;
		}

		if (codecMode_ == CODEC_MODE_BOTH || codecMode_ == CODEC_MODE_DECODER)
		{
			int errCode = OPUS_OK;
			LOG4CXX_INFO(logger_, "Initializing Opus audio decoder...");
			LOG4CXX_INFO(logger_, "Audio frequency: " << audioFormat_.frequency << "kHz");
			LOG4CXX_INFO(logger_, "Audio number of channels: " << audioFormat_.numChannels);

			audioDecoder_ = opus_decoder_create(audioFormat_.frequency, audioFormat_.numChannels, &errCode);

			if (errCode != OPUS_OK)
			{
				LOG4CXX_ERROR(logger_, "Could not initialize Opus decoder: " << opus_strerror(errCode));
				deinit();
				return false;
			}

			LOG4CXX_INFO(logger_, "Initialized Opus audio decoder");
			isDecoderInit = true;
		}

		isInit_ = codecMode_ == CODEC_MODE_BOTH ? isDecoderInit && isEncoderInit : codecMode_ == CODEC_MODE_ENCODER ? isEncoderInit : isDecoderInit;
	}

	return isInit();
}

void HoloCodecOpus::deinit()
{
	if (audioEncoder_)
	{
		opus_encoder_destroy(audioEncoder_);
		audioEncoder_ = nullptr;
	}

	if (audioDecoder_)
	{
		opus_decoder_destroy(audioDecoder_);
		audioEncoder_ = nullptr;
	}
}

void HoloCodecOpus::encode(boost::shared_ptr<std::vector<unsigned char>> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut)
{
	//get the size of the raw buffer we just got from the audio input, add any overflow size to that
	int rawSize = (rawData->size() + prevOverflowBuffer_.size());

	//the number of frames is the raw size divided by the length of a raw frame
	//overflow for next cycle, because we won't always have divisible number of frames to jam in there
	const int numFrames = rawSize / rawFrameLength_;
	const int overflow = rawSize % rawFrameLength_;

	//the final data that will be sent over network, this has a header where first int is # of frames packed
	//followed by n number of ints describing the encoded length of each frame (where n is # of frames)
	//after the header, encoded audio frames are packed one after another
	const int finalDataHeaderSize = sizeof(int)*(numFrames + 1);
	//std::vector<unsigned char> finalData(finalDataHeaderSize);
	encodeOut->resize(finalDataHeaderSize);

	//set first int in the header to the number of frames we will encode
	((int*)encodeOut->data())[0] = numFrames;

	//create the raw buffer and copy overflow + chunk into it
	unsigned char * rawBuffer = new unsigned char[rawSize];
	if (!prevOverflowBuffer_.empty())
	{
		memcpy(rawBuffer, &prevOverflowBuffer_[0], prevOverflowBuffer_.size());
		memcpy(rawBuffer + prevOverflowBuffer_.size(), rawData->data(), rawData->size());
	}
	else
		memcpy(rawBuffer, rawData->data(), rawData->size());

	//the buffer where we will store the encoded data on a per frame basis, allocated to the maximum frame length
	unsigned char *encodeBuffer = new unsigned char[rawFrameLength_];

	unsigned char *rawPtr = rawBuffer;
	for (int i = 0; i < numFrames; i++)
	{
		//encode a frame, returns the output number of bytes (encoded buffer length for this particular frame)
		int realLength = opus_encode(audioEncoder_, (const opus_int16*)(rawPtr), HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE, encodeBuffer, rawFrameLength_);

		//set the finaldata buffer size
		((int*)encodeOut->data())[i + 1] = realLength;

		//add a frame to the end of finalData
		encodeOut->insert(encodeOut->end(), &encodeBuffer[0], &encodeBuffer[realLength]);
		rawPtr += rawFrameLength_;
	}

	//get rid of our previous overflow and fill it with current overflow, for next cycle
	prevOverflowBuffer_.clear();
	if (overflow > 0)
		prevOverflowBuffer_.insert(prevOverflowBuffer_.begin(), &rawBuffer[rawSize - overflow], &rawBuffer[rawSize]);

	delete[] encodeBuffer;
	delete[] rawBuffer;
}

void HoloCodecOpus::decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<std::vector<unsigned char>>& decodeOut)
{
	//get the number of frames to decode from first int of rx data
	const int numFrames = ((int*)encodedStream->data())[0];

	//set the data pointer to first byte of encoded data (AKA skip header)
	unsigned char * encodedDataPtr = encodedStream->data() + (sizeof(int)*(numFrames + 1));

	//create a chunk to store numFrames of raw decoded data (48000 * 2chan * 16-bit)
	decodeOut->resize(rawFrameLength_*numFrames);
	//AlignedAudioChunk chunk(remoteSessionInfo_.audioFormat, audioEncBuffLen*numFrames);
	unsigned char * decodedDataPtr = decodeOut->data();

	//decode each frame and fill chunk to the end
	for (int i = 0; i < numFrames; i++)
	{
		int encodedFrameLen = ((int*)encodedStream->data())[i + 1];
		int ret = opus_decode(audioDecoder_, encodedDataPtr, encodedFrameLen, (opus_int16*)decodedDataPtr, HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE, 0);
		encodedDataPtr += encodedFrameLen;
		decodedDataPtr += rawFrameLength_;
	}
}

#endif