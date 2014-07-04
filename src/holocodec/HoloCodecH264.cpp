#include "HoloCodecH264.hpp"

#include <future>

extern "C"
{
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include <opencv2/opencv.hpp>

#ifdef TRACE_LOG_ENABLED
#include <ctime>
#include <chrono>
#endif

using namespace holo;
using namespace holo::codec;

HoloCodecH264::HoloCodecH264() :
	isInit_(false),
	args_(),
	encoder_(NULL),
	decoder_(NULL),
	encoderCtx_(NULL),
	decoderCtx_(NULL),
	encodeInFrame_(NULL),
	decodeOutFrame_(NULL),
	swScaleEncodeCtx_(NULL),
	swScaleDecodeCtx_(NULL),
	encodePacket_(),
	decodePacket_(),
	encodeBufferSize_(0)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.h264");

	AVRational rat;
	rat.num = HOLO_CODEC_H264_DEFAULT_TIMEBASE_NUM;
	rat.den = HOLO_CODEC_H264_DEFAULT_TIMEBASE_DEN;

	args_.bitRate = HOLO_CODEC_H264_DEFAULT_BITRATE;
	args_.width = HOLO_CODEC_H264_DEFAULT_WIDTH;
	args_.height = HOLO_CODEC_H264_DEFAULT_HEIGHT;
	args_.gopSize = HOLO_CODEC_H264_DEFAULT_GOPSIZE;
	args_.maxBFrames = HOLO_CODEC_H264_DEFAULT_MAXBFRAMES;
	args_.timeBase = rat;
	args_.pixelFormat = HOLO_CODEC_H264_DEFAULT_PIXELFMT;
	args_.crf = HOLO_CODEC_H264_DEFAULT_CRF;
	args_.zCompressionLevel = HOLO_CODEC_H264_DEFAULT_ZCOMPRESSIONLEVEL;

	LOG4CXX_DEBUG(logger_, "HoloCodecH264 object instantiated with default values");
}

HoloCodecH264::HoloCodecH264(HoloCodecH264Args args) :
	isInit_(false),
	args_(args),
	encoder_(NULL),
	decoder_(NULL),
	encoderCtx_(NULL),
	decoderCtx_(NULL),
	encodeInFrame_(NULL),
	decodeOutFrame_(NULL),
	swScaleEncodeCtx_(NULL),
	swScaleDecodeCtx_(NULL),
	encodePacket_(),
	decodePacket_(),
	encodeBufferSize_(0)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.h264");

	LOG4CXX_DEBUG(logger_, "HoloCodecH264 object instantiated with custom values");
}

HoloCodecH264::~HoloCodecH264()
{
	LOG4CXX_DEBUG(logger_, "Destroying HoloCodecH264 object");
	deinit();
	LOG4CXX_DEBUG(logger_, "HoloCodecH264 object destroyed");
}

bool HoloCodecH264::init(CODEC_MODE encodeOrDecode)
{
	deinit();

	bool encoderIsInit = false;
	bool decoderIsInit = false;

	codecMode_ = encodeOrDecode;

	avcodec_register_all();

	if (codecMode_ == CODEC_MODE_BOTH || codecMode_ == CODEC_MODE_ENCODER)
	{
		LOG4CXX_DEBUG(logger_, "Initializing FFMPEG H264 encoder...");

		encoder_ = avcodec_find_encoder(AV_CODEC_ID_H264);
		if (!encoder_)
		{
			LOG4CXX_ERROR(logger_, "Could not find FFMPEG encoder with ID AV_CODEC_ID_H264");
			return false;
		}

		encoderCtx_ = avcodec_alloc_context3(encoder_);
		if (!encoderCtx_)
		{
			LOG4CXX_ERROR(logger_, "Could not create encoder context");
			return false;
		}

		LOG4CXX_DEBUG(logger_, "Setting H264 encoder settings...");
		encoderCtx_->rc_max_rate = args_.bitRate;
		encoderCtx_->width = args_.width;
		encoderCtx_->height = args_.height;
		encoderCtx_->time_base = args_.timeBase;
		encoderCtx_->gop_size = args_.gopSize;
		encoderCtx_->max_b_frames = args_.maxBFrames;
		encoderCtx_->pix_fmt = args_.pixelFormat;
		//encoderCtx_->thread_count = 2;
		encoderCtx_->delay = 0;
		encoderCtx_->rc_buffer_size = 20000000;

		char crf[33];
		//itoa(args_.crf, crf, 10);

		sprintf(crf, "%d", args_.crf);

		av_opt_set(encoderCtx_->priv_data, "preset", "ultrafast", 0);
		av_opt_set(encoderCtx_->priv_data, "tune", "zerolatency", 0);
		av_opt_set(encoderCtx_->priv_data, "crf", crf, 0);
		
		if (avcodec_open2(encoderCtx_, encoder_, NULL) < 0)
		{
			LOG4CXX_ERROR(logger_, "Could not open H264 encoder from context");
			return false;
		}

		encodeInFrame_ = avcodec_alloc_frame();
		encodeInFrame_->format = args_.pixelFormat;
		encodeInFrame_->width = encoderCtx_->width;
		encodeInFrame_->height = encoderCtx_->height;

		encodeBufferSize_ = av_image_alloc(encodeInFrame_->data, encodeInFrame_->linesize, encoderCtx_->width, encoderCtx_->height, encoderCtx_->pix_fmt, 32);
		if (encodeBufferSize_ <= 0)
		{
			LOG4CXX_ERROR(logger_, "Could not allocate encode frame data buffer");
			return false;
		}

		swScaleEncodeCtx_ = sws_getContext(args_.width, args_.height, AV_PIX_FMT_RGBA, args_.width, args_.height, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
		if (!swScaleEncodeCtx_)
		{
			LOG4CXX_ERROR(logger_, "Could not create swscale context for encoder image color conversion");
			return false;
		}
		
		LOG4CXX_INFO(logger_, "Initialized FFMPEG H264 encoder");

		encoderIsInit = true;
	}

	if (codecMode_ == CODEC_MODE_BOTH || codecMode_ == CODEC_MODE_DECODER)
	{
		LOG4CXX_INFO(logger_, "Initializing FFMPEG H264 decoder...");

		decoder_ = avcodec_find_decoder(AV_CODEC_ID_H264);
		if (!decoder_)
		{
			LOG4CXX_ERROR(logger_, "Could not find FFMPEG decoder with ID AV_CODEC_ID_H264");
			return false;
		}

		decoderCtx_ = avcodec_alloc_context3(decoder_);
		if (!decoderCtx_)
		{
			LOG4CXX_ERROR(logger_, "Could not create H264 decoder context");
			return false;
		}

		if (avcodec_open2(decoderCtx_, decoder_, NULL) < 0)
		{
			LOG4CXX_ERROR(logger_, "Could not open H264 decoder");
			return false;
		}

		decodeOutFrame_ = avcodec_alloc_frame();

		swScaleDecodeCtx_ = sws_getContext(args_.width, args_.height, AV_PIX_FMT_YUV420P, args_.width, args_.height, AV_PIX_FMT_RGBA, SWS_FAST_BILINEAR, NULL, NULL, NULL);
		if (!swScaleDecodeCtx_)
		{
			LOG4CXX_ERROR(logger_, "Could not swscale context for decoder image color conversion");
			return false;
		}

		LOG4CXX_INFO(logger_, "Initialized FFMPEG H264 decoder");

		decoderIsInit = true;
	}

	isInit_ = codecMode_ == CODEC_MODE_BOTH ? decoderIsInit && encoderIsInit : codecMode_ == CODEC_MODE_DECODER ? decoderIsInit : encoderIsInit;

	return isInit();
}

void HoloCodecH264::deinit()
{
	if (isInit_)
	{
		if (codecMode_ == CODEC_MODE_BOTH || codecMode_ == CODEC_MODE_ENCODER)
		{
			LOG4CXX_DEBUG(logger_, "Deinitializing H264 encoder...");

			if (swScaleEncodeCtx_)
			{
				LOG4CXX_DEBUG(logger_, "Freeing encoder swscale context");
				av_freep(&swScaleEncodeCtx_);
			}

			if (encodeInFrame_)
			{
				LOG4CXX_DEBUG(logger_, "Freeing encoder frame data");
				if (encodeInFrame_->data[0])
					av_freep(&encodeInFrame_->data[0]);

				LOG4CXX_DEBUG(logger_, "Freeing encoder frame object");
				avcodec_free_frame(&encodeInFrame_);
			}

			if (encoderCtx_)
			{
				LOG4CXX_DEBUG(logger_, "Closing encoder");
				avcodec_close(encoderCtx_);
				LOG4CXX_DEBUG(logger_, "Freeing encoder context");
				av_freep(&encoderCtx_);
				LOG4CXX_DEBUG(logger_, "Freeing encoder object");
				av_freep(&encoder_);
			}

			LOG4CXX_INFO(logger_, "H264 encoder deinitialized");

		}

		if (codecMode_ == CODEC_MODE_BOTH || codecMode_ == CODEC_MODE_DECODER)
		{

			LOG4CXX_DEBUG(logger_, "Deinitializing H264 decoder...");

			if (swScaleDecodeCtx_)
			{
				LOG4CXX_DEBUG(logger_, "Freeing decoder swscale context");
				av_freep(&swScaleDecodeCtx_);
			}

			if (decodeOutFrame_)
			{
				LOG4CXX_DEBUG(logger_, "Freeing decoder frame object");
				avcodec_free_frame(&decodeOutFrame_);
			}

			if (decoderCtx_)
			{
				LOG4CXX_DEBUG(logger_, "Closing decoder");
				avcodec_close(decoderCtx_);
				LOG4CXX_DEBUG(logger_, "Freeing encoder context");
				av_freep(&decoderCtx_);
				LOG4CXX_DEBUG(logger_, "Freeing encoder object");
				av_freep(&decoder_);
			}

			LOG4CXX_INFO(logger_, "H264 decoder deinitialized");
		}
	}
	
	isInit_ = false;
}

void HoloCodecH264::encode(boost::shared_ptr<HoloRGBAZMat> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut)
{
	auto futureColor = std::async(std::launch::async, &HoloCodecH264::encodeColorFrame, this, std::ref(rawData->rgba));

	auto futureZ = std::async(std::launch::async, &HoloCodecH264::encodeZFrame, this, std::ref(rawData->z));

	auto color = (boost::shared_ptr<std::vector<unsigned char>>)futureColor.get();
	auto z = (boost::shared_ptr<std::vector<unsigned char>>)futureZ.get();

	if (color && z)
	{
		encodeOut = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>(z->size() + color->size() + sizeof(int)* 2));
		*(int*)&(*encodeOut)[0] = color->size();
		*(int*)&(*encodeOut)[4] = z->size();

		std::copy(color->begin(), color->end(), encodeOut->begin() + sizeof(int)* 2);
		std::copy(z->begin(), z->end(), encodeOut->begin() + color->size() + sizeof(int)* 2);
	}
	else
		encodeOut = nullptr;
}

void HoloCodecH264::decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloRGBAZMat>& decodeOut)
{
	int colorSize = *(int*)&(*encodedStream)[0];
	int zSize = *(int*)&(*encodedStream)[4];

	auto colorDecodeFunc = std::bind(&HoloCodecH264::decodeColorFrame, this, encodedStream->begin() += (sizeof(int)* 2), std::placeholders::_1);
	auto futureColor = std::async(std::launch::async, colorDecodeFunc, encodedStream->begin() += (sizeof(int)* 2 + colorSize));

	auto zDecodeFunc = std::bind(&HoloCodecH264::decodeZFrame, this, encodedStream->begin() += (sizeof(int)* 2 + colorSize), std::placeholders::_1);
	auto futureZ = std::async(std::launch::async, zDecodeFunc, encodedStream->end());

	auto color = (boost::shared_ptr<cv::Mat>)futureColor.get();
	auto z = (boost::shared_ptr<cv::Mat>)futureZ.get();

	if (color && z)
	{
		decodeOut = boost::shared_ptr<HoloRGBAZMat>(new HoloRGBAZMat);
		decodeOut->rgba = *color;
		decodeOut->z = *z;
	}
	else
		decodeOut = nullptr;
}

boost::shared_ptr<std::vector<unsigned char>> HoloCodecH264::encodeColorFrame(cv::Mat& rgba)
{
	av_init_packet(&encodePacket_);
	encodePacket_.data = NULL;
	encodePacket_.size = 0;
	int gotOutput = 0;

	uint8_t * rgbaBuf[4];
	rgbaBuf[0] = rgba.data;
	rgbaBuf[1] = NULL;
	rgbaBuf[2] = NULL;
	rgbaBuf[3] = NULL;

	int stride[4] = { args_.width << 2, 0, 0, 0 };

	sws_scale(swScaleEncodeCtx_, rgbaBuf, stride, 0, rgba.rows, encodeInFrame_->data, encodeInFrame_->linesize);

	//memcpy(encodeInFrame_->data[0], yuv.data, 640 * 480);
	//memcpy(encodeInFrame_->data[1], yuv.data + 640*480, 76800);
	//memcpy(encodeInFrame_->data[2], yuv.data + 640 * 480 + 76800, 76800);

	encodeInFrame_->pts = nextPTS();

#ifdef TRACE_LOG_ENABLED
	auto startTime = std::chrono::system_clock::now();
#endif

	if (avcodec_encode_video2(encoderCtx_, &encodePacket_, encodeInFrame_, &gotOutput) != 0)
	{
		LOG4CXX_DEBUG(logger_, "Failed to encode h.264 color frame");
		return nullptr;
	}

#ifdef TRACE_LOG_ENABLED
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
	LOG4CXX_TRACE(logger_, "h.264 encoded a " << encodePacket_.size << " byte color frame in " << duration.count() << "ms");
#endif

	if (gotOutput > 0)
	{
		//std::cout << "Got output: " << gotOutput;
		auto encodeOut = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>(encodePacket_.size));
		memcpy(encodeOut->data(), encodePacket_.data, encodePacket_.size);
		av_free_packet(&encodePacket_);
		return encodeOut;
	}
	else
		return nullptr;
}

boost::shared_ptr<cv::Mat> HoloCodecH264::decodeColorFrame(std::vector<unsigned char>::iterator dataBegin, std::vector<unsigned char>::iterator dataEnd)
{
	auto encodedRGBA = std::vector<unsigned char>(dataBegin, dataEnd);
	auto rgba = boost::shared_ptr<cv::Mat>(new cv::Mat(args_.height, args_.width, CV_8UC4));
	int gotPicture = 0;
	av_new_packet(&decodePacket_, encodedRGBA.size());

	memcpy(decodePacket_.data, encodedRGBA.data(), decodePacket_.size);

#ifdef TRACE_LOG_ENABLED
	auto startTime = std::chrono::system_clock::now();
#endif

	int len = avcodec_decode_video2(decoderCtx_, decodeOutFrame_, &gotPicture, &decodePacket_);

#ifdef TRACE_LOG_ENABLED
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
	LOG4CXX_TRACE(logger_, "h.264 decoded a " << len << " byte color frame in " << duration.count() << "ms");
#endif

	av_free_packet(&decodePacket_);

	if (gotPicture > 0)
	{
		//std::cout << "Got picture(s): " << gotPicture << std::endl;

		uint8_t *rgbaBuf[4];
		rgbaBuf[0] = rgba->data;
		rgbaBuf[1] = NULL;
		rgbaBuf[2] = NULL;
		rgbaBuf[3] = NULL;

		int stride[4] = { args_.width << 2, 0, 0, 0 };

		sws_scale(swScaleDecodeCtx_, decodeOutFrame_->data, decodeOutFrame_->linesize, 0, rgba->rows, rgbaBuf, stride);

		return rgba;
	}
	else
		return nullptr;
}

boost::shared_ptr<std::vector<unsigned char>> HoloCodecH264::encodeZFrame(cv::Mat& z)
{
	auto encodeData = boost::shared_ptr <std::vector<unsigned char>>(new std::vector<unsigned char>());
	std::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(args_.zCompressionLevel);
	compression_params.push_back(CV_IMWRITE_PNG_STRATEGY);
	compression_params.push_back(CV_IMWRITE_PNG_STRATEGY_RLE);

#ifdef TRACE_LOG_ENABLED
	auto startTime = std::chrono::system_clock::now();
#endif
	if (cv::imencode(".png", z, *encodeData, compression_params))
	{
#ifdef TRACE_LOG_ENABLED
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
		LOG4CXX_TRACE(logger_, "h.264 encoded a " << encodeData->size() << " byte png z-frame in " << duration.count() << "ms");
#endif
		return encodeData;
	}
	else
		return nullptr;
}

boost::shared_ptr<cv::Mat> HoloCodecH264::decodeZFrame(std::vector<unsigned char>::iterator dataBegin, std::vector<unsigned char>::iterator dataEnd)
{
	auto encodedRGBA = std::vector<unsigned char>(dataBegin, dataEnd);

#ifdef TRACE_LOG_ENABLED
	auto startTime = std::chrono::system_clock::now();
#endif

	auto decodeMat = boost::shared_ptr<cv::Mat>(new cv::Mat(cv::imdecode(encodedRGBA, CV_LOAD_IMAGE_ANYDEPTH)));

#ifdef TRACE_LOG_ENABLED
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
	LOG4CXX_TRACE(logger_, "h.264 decoded a " << encodedRGBA.size() << " byte png z-frame in " << duration.count() << "ms");
#endif

	if (!decodeMat->empty())
		return decodeMat;
	else
		return nullptr;
}
