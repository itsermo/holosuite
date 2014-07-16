#pragma once
#ifdef ENABLE_HOLO_AUDIO

#include "IHoloCodec.hpp"
#include <opus.h>
#include <vector>
#include <opus.h>

namespace holo
{
	namespace codec
	{
		class HoloCodecOpus : public IHoloCodec<std::vector<unsigned char>>
		{
		public:
			HoloCodecOpus();
			HoloCodecOpus(HoloAudioFormat audioFormat, int bitRate);
			~HoloCodecOpus();

			bool init(CODEC_MODE codecMode_);
			void deinit();

			void encode(boost::shared_ptr<std::vector<unsigned char>> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut);
			void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<std::vector<unsigned char>>& decodeOut);

			CODEC_MODE getcodecMode() { return codecMode_; }

			bool isInit() { return isInit_; }

		private:
			int rawFrameLength_;
			std::vector<unsigned char> prevOverflowBuffer_;
			bool isInit_;
			int bitRate_;
			CODEC_MODE codecMode_;
			HoloAudioFormat audioFormat_;

			OpusEncoder * audioEncoder_;
			OpusDecoder * audioDecoder_;

			log4cxx::LoggerPtr logger_;
		};
	}
}

#endif