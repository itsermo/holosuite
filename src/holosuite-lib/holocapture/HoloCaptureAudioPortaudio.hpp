#pragma once
#ifdef ENABLE_HOLO_AUDIO
#include "../holocommon/CommonDefs.hpp"
#include "IHoloCaptureAudio.hpp"
#include <portaudio.h>

namespace holo
{
	namespace capture
	{
		class HoloCaptureAudioPortaudio : public IHoloCaptureAudio
		{
		public:
			HoloCaptureAudioPortaudio();
			HoloCaptureAudioPortaudio(HoloAudioFormat audioFormat);
			~HoloCaptureAudioPortaudio();
			
			bool init(int which);

			bool isOpen() {
				return isInit_;
			}

			void waitAndGetNextChunk(std::vector<unsigned char>& audioOut);
			void deinit();

			virtual std::vector<std::string> enumerateDevices();

			virtual HoloAudioFormat getAudioFormat() {
				return audioFormat_;
			}

		private:
			//PaHostApiTypeId getHostType();

			PaStreamParameters streamParameters_;
			PaStream *audioStream_;

			bool isInit_;
			HoloAudioFormat audioFormat_;
			log4cxx::LoggerPtr logger_;
		};
	};
};

#endif