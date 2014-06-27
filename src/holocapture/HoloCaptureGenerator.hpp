#pragma once
#include "IHoloCapture.hpp"
#include "HoloCaptureOpenNI2.hpp"
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

			static std::unique_ptr<IHoloCapture> fromOpenNI(unsigned int rgbWidth,
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

			static std::unique_ptr<IHoloCapture> fromOpenNI(std::string filePath)
			{
				return std::unique_ptr<IHoloCapture>(new HoloCaptureOpenNI2(filePath));
			}

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