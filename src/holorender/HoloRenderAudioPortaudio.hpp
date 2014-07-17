#pragma once
#ifdef ENABLE_HOLO_AUDIO
#include "IHoloRenderAudio.hpp"
#include <portaudio.h>

namespace holo
{
	namespace render
	{
		class HoloRenderAudioPortaudio : public IHoloRenderAudio
		{
		public:
			HoloRenderAudioPortaudio();
			HoloRenderAudioPortaudio(HoloAudioFormat audioFormat);
			~HoloRenderAudioPortaudio();

			bool init(int which);
			void deinit();
			void playSoundBuffer(boost::shared_ptr<std::vector<uchar>> && soundBuffer);
			std::vector<std::string> enumerateDevices();

			bool isInit() { return isInit_; }

			void* getContext() { return (void*)audioStream_; }

		private:
			PaHostApiTypeId getHostType();

			bool isInit_;

			PaStream *audioStream_;
			PaStreamParameters streamParameters_;
			HoloAudioFormat audioFormat_;

			log4cxx::LoggerPtr logger_;
		};
	}
}
#endif