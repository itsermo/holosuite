#pragma once

#include "IHoloCapture.hpp"
#include "HoloCaptureInfo.hpp"

#include <OpenNI.h>
#include <log4cxx/log4cxx.h>

#include <log4cxx/logger.h>



namespace holo
{
	namespace capture
	{
		class HoloCaptureOpenNI2 : public IHoloCapture
		{

		public:
			HoloCaptureOpenNI2();
			HoloCaptureOpenNI2(std::string filePath);
			HoloCaptureOpenNI2(unsigned int rgbWidth,
				unsigned int rgbHeight,
				float rgbFPS,
				unsigned int zWidth,
				unsigned int zHeight,
				float zFPS);
			~HoloCaptureOpenNI2();
			virtual bool init(int which = 0);

			virtual bool isOpen() { return isOpen_; };
			virtual void waitAndGetNextFrame(cv::Mat& rgbaImage, cv::Mat& zImage);
			virtual void waitAndGetNextPointCloud(HoloCloudPtr& pointCloud);
			virtual void deinit();
			std::list<std::string> enumerateDevices();
			virtual HoloCaptureInfo getCaptureInfo()
			{
				HoloCaptureInfo info;
				info.rgbaWidth = rgbWidth_;
				info.rgbaHeight = rgbHeight_;
				info.rgbFPS = rgbFPS_;
				info.rgbHOV = 0;
				info.rgbVOV = 0;
				info.zWidth = zWidth_;
				info.zHeight = zHeight_;
				info.zFPS = zFPS_;
				info.zHOV = hov_ * 180.0f / M_PI;
				info.zVOV = vov_ * 180.0f / M_PI;

				return info;
			}

		private:
			bool initOpenNI2();
			void deinitOpenNI2();
			openni::Device device_;
			openni::VideoStream depthStream_;
			openni::VideoStream colorStream_;

			cv::Mat rgbImage_;
			cv::Mat rgbaImage_;
			cv::Mat depthImage_;

			HoloCloudPtr pointCloud_;
			HoloCloudPtr createNewCloud();

			unsigned int rgbWidth_;
			unsigned int rgbHeight_;
			unsigned int zWidth_;
			unsigned int zHeight_;
			float rgbFPS_;
			float zFPS_;
			float hov_;
			float vov_;
			bool isOpen_;
			bool isOpenNI2Init_;

			WorldConvertCache worldConvertCache_;

			std::string filePath_;

			log4cxx::LoggerPtr logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.openni2");
		};
	}
}