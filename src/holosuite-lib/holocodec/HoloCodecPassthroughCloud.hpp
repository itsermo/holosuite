#pragma once

#include "IHoloCodec.hpp"

namespace holo
{
	namespace codec
	{

		class HoloCodecPassthroughCloud : public IHoloCodec<HoloCloud>
		{

		public:
			HoloCodecPassthroughCloud();
			HoloCodecPassthroughCloud(int width, int height);
			~HoloCodecPassthroughCloud();

			virtual bool init(CODEC_MODE codecMode_);
			virtual void deinit();

			virtual void encode(boost::shared_ptr<HoloCloud> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut);
			virtual void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut);

			virtual CODEC_MODE getcodecMode() { return codecMode_; }

			virtual bool isInit() { return isInit_; }

		private:
			int width_;
			int height_;
			bool isInit_;
			CODEC_MODE codecMode_;

			log4cxx::LoggerPtr logger_;
		};
	}
}