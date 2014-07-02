#pragma once

#include "../common/CommonDefs.hpp"
#include "IHoloCodec.hpp"

extern "C" {
#include <libavutil/mem.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#define HOLO_CODEC_H264_DEFAULT_BITRATE 512000
#define HOLO_CODEC_H264_DEFAULT_WIDTH HOLO_CAPTURE_DEFAULT_Z_WIDTH
#define HOLO_CODEC_H264_DEFAULT_HEIGHT HOLO_CAPTURE_DEFAULT_Z_HEIGHT
#define HOLO_CODEC_H264_DEFAULT_TIMEBASE_DEN 30
#define HOLO_CODEC_H264_DEFAULT_TIMEBASE_NUM 1
#define HOLO_CODEC_H264_DEFAULT_GOPSIZE HOLO_CODEC_H264_DEFAULT_TIMEBASE_DEN
#define HOLO_CODEC_H264_DEFAULT_MAXBFRAMES 0
#define HOLO_CODEC_H264_DEFAULT_PIXELFMT AV_PIX_FMT_YUV420P
#define HOLO_CODEC_H264_DEFAULT_CRF 20
#define HOLO_CODEC_H264_DEFAULT_ZCOMPRESSIONLEVEL 7

namespace holo
{
	namespace codec
	{
		struct HoloCodecH264Args
		{
			int bitRate;
			int width;
			int height;
			AVRational timeBase;
			int gopSize;
			int maxBFrames;
			AVPixelFormat pixelFormat;
			int crf;
			int zCompressionLevel;
		};

		class HoloCodecH264 : public IHoloCodec<HoloRGBAZMat>
		{
		public:
			HoloCodecH264();
			HoloCodecH264(HoloCodecH264Args args);
			~HoloCodecH264();
			
			virtual bool init(CODEC_MODE encodeOrDecode);
			virtual void deinit();
			virtual bool isInit() { return isInit_; }

			virtual void encode(boost::shared_ptr<HoloRGBAZMat> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut);
			virtual void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloRGBAZMat>& decodeOut);

			virtual CODEC_MODE getcodecMode() { return codecMode_; }

			int nextPTS()
			{
				static int static_pts = 0;
				return static_pts++;
			}

		private:
			
			bool isInit_;

			boost::shared_ptr<std::vector<unsigned char>> encodeColorFrame(cv::Mat& rgba);
			boost::shared_ptr<cv::Mat> decodeColorFrame(std::vector<unsigned char>::iterator dataBegin, std::vector<unsigned char>::iterator dataEnd);

			boost::shared_ptr<std::vector<unsigned char>> encodeZFrame(cv::Mat& z);
			boost::shared_ptr<cv::Mat> decodeZFrame(std::vector<unsigned char>::iterator dataBegin, std::vector<unsigned char>::iterator dataEnd);

			SwsContext * swScaleEncodeCtx_;
			SwsContext * swScaleDecodeCtx_;

			AVCodec *encoder_;
			AVCodec *decoder_;
			
			AVCodecContext *encoderCtx_;
			AVCodecContext *decoderCtx_;

			AVFrame *encodeInFrame_;
			AVFrame *decodeOutFrame_;

			AVPacket decodePacket_;
			AVPacket encodePacket_;

			HoloCodecH264Args args_;

			int encodeBufferSize_;

			CODEC_MODE codecMode_;

			log4cxx::LoggerPtr logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.h264");

		};
	}
}
