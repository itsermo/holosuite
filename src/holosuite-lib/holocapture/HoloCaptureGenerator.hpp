#pragma once
#include "IHoloCapture.hpp"
#include "HoloCaptureOpenNI2.hpp"

#ifdef ENABLE_HOLO_AUDIO
#include "HoloCaptureAudioPortaudio.hpp"
#endif

#include <memory>

namespace holo
{
	namespace capture
	{
		class HoloCaptureGenerator
		{
		public:
			HoloCaptureGenerator();
			~HoloCaptureGenerator();

			static std::unique_ptr<IHoloCapture> fromOpenNI()
			{
				return std::unique_ptr<IHoloCapture>(new HoloCaptureOpenNI2());
			}

			static std::unique_ptr<IHoloCapture> fromOpenNI2(unsigned int rgbWidth,
				unsigned int rgbHeight,
				float rgbFPS,
				unsigned int zWidth,
				unsigned int zHeight,
				float zFPS)
			{
				return std::unique_ptr<IHoloCapture>(new HoloCaptureOpenNI2(rgbWidth,
					rgbHeight,
					rgbFPS,
					zWidth,
					zHeight,
					zFPS));
			}

			static std::unique_ptr<IHoloCapture> fromOpenNI2(std::string filePath)
			{
				return std::unique_ptr<IHoloCapture>(new HoloCaptureOpenNI2(filePath));
			}

			static std::unique_ptr<IHoloCapture> fromOpenNI2()
			{
				return std::unique_ptr<IHoloCapture>(new HoloCaptureOpenNI2());
			}

#ifdef ENABLE_HOLO_AUDIO
			static std::unique_ptr<IHoloCaptureAudio> fromPortaudio()
			{
				return std::unique_ptr<IHoloCaptureAudio>(new HoloCaptureAudioPortaudio());
			}

			static std::unique_ptr<IHoloCaptureAudio> fromPortaudio(HoloAudioFormat audioFormat)
			{
				return std::unique_ptr<IHoloCaptureAudio>(new HoloCaptureAudioPortaudio(audioFormat));
			}
#endif

		private:

		};

		HoloCaptureGenerator::HoloCaptureGenerator()
		{
		}

		HoloCaptureGenerator::~HoloCaptureGenerator()
		{
		}
	}
}
