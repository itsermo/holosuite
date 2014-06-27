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
			~HoloCodecPassthroughCloud();

			virtual bool init(CODEC_TYPE codecType_);
			virtual void deinit();

			virtual void encode(boost::shared_ptr<HoloCloud> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut);
			virtual void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut);

			virtual CODEC_TYPE getCodecType() { return codecType_; }

			virtual bool isInit() { return isInit_; }

		private:
			bool isInit_;
			CODEC_TYPE codecType_;

			log4cxx::LoggerPtr logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.passthroughcloud");
		};
	}
}