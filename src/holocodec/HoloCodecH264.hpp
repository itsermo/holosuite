#pragma once

#include "../common/CommonDefs.hpp"
#include "IHoloCodec.hpp"

extern "C" {
#include <libavutil/mem.h>
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
}

#define HOLO_CODEC_H264_DEFAULT_BITRATE 400000
#define HOLO_CODEC_H264_DEFAULT_WIDTH HOLO_CAPTURE_DEFAULT_Z_WIDTH
#define HOLO_CODEC_H264_DEFAULT_HEIGHT HOLO_CAPTURE_DEFAULT_Z_HEIGHT
#define HOLO_CODEC_H264_DEFAULT_TIMEBASE_DEN 30
#define HOLO_CODEC_H264_DEFAULT_TIMEBASE_NUM 1
#define HOLO_CODEC_H264_DEFAULT_GOPSIZE 10
#define HOLO_CODEC_H264_DEFAULT_MAXBFRAMES 1
#define HOLO_CODEC_H264_DEFAULT_PIXELFMT AV_PIX_FMT_YUV420P

namespace holo
{
	namespace codec
	{
		class HoloCodecH264 : public IHoloCodec<HoloRGBAZMat>
		{
		public:
			HoloCodecH264();
			HoloCodecH264(int bitRate, int width, int height, AVRational timeBase, int gopSize, int maxBFrames, AVPixelFormat pixFmt);
			~HoloCodecH264();
			
			virtual bool init(CODEC_TYPE encodeOrDecode);
			virtual void deinit();
			virtual bool isInit() { return isInit_; }

			virtual void encode(boost::shared_ptr<HoloRGBAZMat> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut);
			virtual void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloRGBAZMat>& decodeOut);

			virtual CODEC_TYPE getCodecType() { return codecType_; }

			int nextPTS()
			{
				static int static_pts = 0;
				return static_pts++;
			}

		private:
			
			bool isInit_;

			boost::shared_ptr<std::vector<unsigned char>> encodeColorFrame(cv::Mat rgba);
			boost::shared_ptr<cv::Mat> decodeColorFrame(std::vector<unsigned char>::iterator dataBegin, std::vector<unsigned char>::iterator dataEnd);

			boost::shared_ptr<std::vector<unsigned char>> encodeZFrame(cv::Mat z);
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

			int bitRate_;
			int width_;
			int height_;
			AVRational timeBase_;
			int gopSize_;
			int maxBFrames_;
			AVPixelFormat pixelFormat_;

			int encodeBufferSize_;

			CODEC_TYPE codecType_;

			log4cxx::LoggerPtr logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.h264");

		};
	}
}
