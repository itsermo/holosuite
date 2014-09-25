#pragma once

#include "../holocommon/CommonDefs.hpp"
#include <boost/shared_ptr.hpp>
#include <vector>

namespace holo
{
	namespace codec
	{
		template <class T>
		class IHoloCodec 
		{
		public:
			virtual ~IHoloCodec<T>() = 0;
			virtual bool init(CODEC_MODE encodeOrDecode) = 0;
			virtual void deinit() = 0;
			virtual bool isInit() = 0;

			virtual void encode(boost::shared_ptr<T> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut) = 0;
			virtual void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<T>& decodeOut) = 0;

			virtual CODEC_MODE getcodecMode() = 0;
		};

		template<class T>
		inline IHoloCodec<T>::~IHoloCodec() { }
	}
}
