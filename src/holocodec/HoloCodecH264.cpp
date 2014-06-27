#include "HoloCodecH264.hpp"

#include <future>

extern "C"
{
#include <libavutil/opt.h>
#include <libavutil/imgutils.h>
}

#include <opencv2/opencv.hpp>

using namespace holo;
using namespace holo::codec;

HoloCodecH264::HoloCodecH264() :
	isInit_(false),
	bitRate_(HOLO_CODEC_H264_DEFAULT_BITRATE),
	width_(HOLO_CODEC_H264_DEFAULT_WIDTH),
	height_(HOLO_CODEC_H264_DEFAULT_HEIGHT),
	gopSize_(HOLO_CODEC_H264_DEFAULT_GOPSIZE),
	maxBFrames_(HOLO_CODEC_H264_DEFAULT_MAXBFRAMES),
	pixelFormat_(HOLO_CODEC_H264_DEFAULT_PIXELFMT),
	encoder_(NULL),
	decoder_(NULL),
	encoderCtx_(NULL),
	decoderCtx_(NULL),
	encodeInFrame_(NULL),
	decodeOutFrame_(NULL),
	swScaleEncodeCtx_(NULL),
	swScaleDecodeCtx_(NULL),
	encodeBufferSize_(0)
{

	AVRational rat;
	rat.den = HOLO_CODEC_H264_DEFAULT_TIMEBASE_DEN;
	rat.num = HOLO_CODEC_H264_DEFAULT_TIMEBASE_NUM;

	timeBase_ = rat;

	logger_->debug("HoloCodecH264 object instantiated with default values");

}

HoloCodecH264::HoloCodecH264(int bitRate, int width, int height, AVRational timeBase, int gopSize, int maxBFrames, AVPixelFormat pixFmt) :
	isInit_(false),
	bitRate_(bitRate),
	width_(width),
	height_(height),
	timeBase_(timeBase),
	gopSize_(gopSize),
	maxBFrames_(maxBFrames),
	pixelFormat_(pixFmt),
	encoder_(NULL),
	decoder_(NULL),
	encoderCtx_(NULL),
	decoderCtx_(NULL),
	encodeInFrame_(NULL),
	decodeOutFrame_(NULL),
	swScaleEncodeCtx_(NULL),
	swScaleDecodeCtx_(NULL),
	encodeBufferSize_(0)
{
	logger_->debug("HoloCodecH264 object instantiated with custom values");
}

HoloCodecH264::~HoloCodecH264()
{
	logger_->debug("Destroying HoloCodecH264 object");
	deinit();
	logger_->debug("HoloCodecH264 object destroyed");
}

bool HoloCodecH264::init(CODEC_TYPE encodeOrDecode)
{
	deinit();

	bool encoderIsInit = false;
	bool decoderIsInit = false;

	codecType_ = encodeOrDecode;

	avcodec_register_all();

	if (codecType_ == CODEC_TYPE_BOTH || codecType_ == CODEC_TYPE_ENCODER)
	{
		logger_->debug("Initializing FFMPEG H264 encoder...");

		encoder_ = avcodec_find_encoder(AV_CODEC_ID_H264);
		if (!encoder_)
		{
			logger_->error("Could not find FFMPEG encoder with ID AV_CODEC_ID_H264");
			return false;
		}

		encoderCtx_ = avcodec_alloc_context3(encoder_);
		if (!encoderCtx_)
		{
			logger_->error("Could not create encoder context");
			return false;
		}

		logger_->debug("Setting H264 encoder settings...");
		encoderCtx_->bit_rate = bitRate_;
		encoderCtx_->width = width_;
		encoderCtx_->height = height_;
		encoderCtx_->time_base = timeBase_;
		encoderCtx_->gop_size = gopSize_;
		encoderCtx_->max_b_frames = maxBFrames_;
		encoderCtx_->pix_fmt = pixelFormat_;

		av_opt_set(encoderCtx_->priv_data, "preset", "ultrafast", 0);
		av_opt_set(encoderCtx_->priv_data, "tune", "zerolatency", 0);
		av_opt_set(encoderCtx_->priv_data, "qp", "4", 0);

		if (avcodec_open2(encoderCtx_, encoder_, NULL) < 0)
		{
			logger_->error("Could not open H264 encoder from context");
			return false;
		}

		encodeInFrame_ = avcodec_alloc_frame();
		encodeInFrame_->format = pixelFormat_;
		encodeInFrame_->width = encoderCtx_->width;
		encodeInFrame_->height = encoderCtx_->height;

		encodeBufferSize_ = av_image_alloc(encodeInFrame_->data, encodeInFrame_->linesize, encoderCtx_->width, encoderCtx_->height, encoderCtx_->pix_fmt, 32);
		if (encodeBufferSize_ <= 0)
		{
			logger_->error("Could not allocate encode frame data buffer");
			return false;
		}

		swScaleEncodeCtx_ = sws_getContext(width_, height_, AV_PIX_FMT_RGBA, width_, height_, AV_PIX_FMT_YUV420P, SWS_FAST_BILINEAR, NULL, NULL, NULL);
		if (!swScaleEncodeCtx_)
		{
			logger_->error("Could not create swscale context for encoder image color conversion");
			return false;
		}
		
		logger_->info("Initialized FFMPEG H264 encoder");

		encoderIsInit = true;
	}

	if (codecType_ == CODEC_TYPE_BOTH || codecType_ == CODEC_TYPE_DECODER)
	{
		logger_->debug("Initializing FFMPEG H264 decoder...");

		decoder_ = avcodec_find_decoder(AV_CODEC_ID_H264);
		if (!decoder_)
		{
			logger_->error("Could not find FFMPEG decoder with ID AV_CODEC_ID_H264");
			return false;
		}

		decoderCtx_ = avcodec_alloc_context3(decoder_);
		if (!decoderCtx_)
		{
			logger_->error("Could not create H264 decoder context");
			return false;
		}

		if (avcodec_open2(decoderCtx_, decoder_, NULL) < 0)
		{
			logger_->error("Could not open H264 decoder");
			return false;
		}

		decodeOutFrame_ = avcodec_alloc_frame();

		swScaleDecodeCtx_ = sws_getContext(width_, height_, AV_PIX_FMT_YUV420P, width_, height_, AV_PIX_FMT_RGBA, SWS_FAST_BILINEAR, NULL, NULL, NULL);
		if (!swScaleDecodeCtx_)
		{
			logger_->error("Could not swscale context for decoder image color conversion");
			return false;
		}

		logger_->info("Initialized FFMPEG H264 decoder");

		decoderIsInit = true;
	}

	isInit_ = codecType_ == CODEC_TYPE_BOTH ? decoderIsInit && encoderIsInit : codecType_ == CODEC_TYPE_DECODER ? decoderIsInit : encoderIsInit;

	return isInit();
}

void HoloCodecH264::deinit()
{
	if (isInit_)
	{

		logger_->debug("Deinitializing H264 codec...");

		if (codecType_ == CODEC_TYPE_BOTH || codecType_ == CODEC_TYPE_ENCODER)
		{
			if (encodeInFrame_)
			{
				logger_->debug("Freeing encoder frame data");
				if (encodeInFrame_->data[0])
					av_freep(&encodeInFrame_->data[0]);

				logger_->debug("Freeing encoder frame object");
				avcodec_free_frame(&encodeInFrame_);
			}

			if (encoderCtx_)
			{
				logger_->debug("Closing encoder");
				avcodec_close(encoderCtx_);
				logger_->debug("Freeing encoder context");
				av_freep(encoderCtx_);
				logger_->debug("Freeing encoder object");
				av_freep(encoder_);
			}
		}

		if (codecType_ == CODEC_TYPE_BOTH || codecType_ == CODEC_TYPE_ENCODER)
		{
			if (decodeOutFrame_)
			{
				logger_->debug("Freeing decoder frame object");
				avcodec_free_frame(&decodeOutFrame_);
			}

			if (decoderCtx_)
			{
				logger_->debug("Closing decoder");
				avcodec_close(decoderCtx_);
				logger_->debug("Freeing encoder context");
				av_freep(decoderCtx_);
				logger_->debug("Freeing encoder object");
				av_freep(decoder_);
			}
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
	int gotOutput;

	uint8_t * rgbaBuf[4];
	rgbaBuf[0] = rgba.data;
	rgbaBuf[1] = NULL;
	rgbaBuf[2] = NULL;
	rgbaBuf[3] = NULL;

	int stride[4] = { width_ << 2, 0, 0, 0 };

	sws_scale(swScaleEncodeCtx_, rgbaBuf, stride, 0, rgba.rows, encodeInFrame_->data, encodeInFrame_->linesize);

	//memcpy(encodeInFrame_->data[0], yuv.data, 640 * 480);
	//memcpy(encodeInFrame_->data[1], yuv.data + 640*480, 76800);
	//memcpy(encodeInFrame_->data[2], yuv.data + 640 * 480 + 76800, 76800);

	encodeInFrame_->pts = nextPTS();

	avcodec_encode_video2(encoderCtx_, &encodePacket_, encodeInFrame_, &gotOutput);

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
	auto rgba = boost::shared_ptr<cv::Mat>(new cv::Mat(height_, width_, CV_8UC4));
	int gotPicture = 0;
	av_new_packet(&decodePacket_, encodedRGBA.size());

	memcpy(decodePacket_.data, encodedRGBA.data(), decodePacket_.size);

	int len = avcodec_decode_video2(decoderCtx_, decodeOutFrame_, &gotPicture, &decodePacket_);

	if (gotPicture > 0)
	{
		//std::cout << "Got picture(s): " << gotPicture << std::endl;

		uint8_t *rgbaBuf[4];
		rgbaBuf[0] = rgba->data;
		rgbaBuf[1] = NULL;
		rgbaBuf[2] = NULL;
		rgbaBuf[3] = NULL;

		int stride[4] = { width_ << 2, 0, 0, 0 };

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
	compression_params.push_back(9);

	if (cv::imencode(".png", z, *encodeData, compression_params))
		return encodeData;
	else
		return nullptr;
}

boost::shared_ptr<cv::Mat> HoloCodecH264::decodeZFrame(std::vector<unsigned char>::iterator dataBegin, std::vector<unsigned char>::iterator dataEnd)
{
	auto encodedRGBA = std::vector<unsigned char>(dataBegin, dataEnd);
	auto decodeMat = boost::shared_ptr<cv::Mat>(new cv::Mat(cv::imdecode(encodedRGBA, CV_LOAD_IMAGE_ANYDEPTH)));

	if (!decodeMat->empty())
		return decodeMat;
	else
		return nullptr;
}
