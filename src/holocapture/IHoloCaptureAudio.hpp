#pragma once

#include "../common/CommonDefs.hpp"
#include <list>

namespace holo
{
	namespace capture
	{
		class IHoloCaptureAudio
		{
		public:
			virtual ~IHoloCaptureAudio() = 0;
			virtual bool init(int which) = 0;
			virtual bool isOpen() = 0;
			virtual void waitAndGetNextChunk(std::vector<unsigned char>& audioOut) = 0;
			virtual void deinit() = 0;
			virtual std::vector<std::string> enumerateDevices() = 0;
			virtual HoloAudioFormat getAudioFormat() = 0;
		};

		inline IHoloCaptureAudio::~IHoloCaptureAudio() { }
	};
};