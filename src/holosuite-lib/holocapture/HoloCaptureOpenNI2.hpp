#pragma once

#include "IHoloCapture.hpp"
#include "HoloCaptureInfo.hpp"
//#include "HoloCaptureOpenNI2Listener.hpp"

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
				float zFPS, bool shouldRecord = false, std::string fileName = "holocapture.oni");
			~HoloCaptureOpenNI2();
			virtual bool init(int which = 0);

			virtual bool isOpen() { return isOpen_; };
			virtual void waitAndGetNextFrame(cv::Mat& rgbaImage, cv::Mat& zImage);
			virtual void waitAndGetNextPointCloud(HoloCloudPtr& pointCloud);
			virtual void deinit();
			std::vector<std::string> enumerateDevices();
			virtual HoloCaptureInfo getCaptureInfo()
			{
				HoloCaptureInfo info;
				info.rgbaWidth = rgbWidth_;
				info.rgbaHeight = rgbHeight_;
				info.rgbFPS = rgbFPS_;
				info.rgbHOV = rgbHOV_ * 180.0f / M_PI;
				info.rgbVOV = rgbVOV_ * 180.0f / M_PI;
				info.zWidth = zWidth_;
				info.zHeight = zHeight_;
				info.zFPS = zFPS_;
				info.zHOV = zHOV_ * 180.0f / M_PI;
				info.zVOV = zVOV_ * 180.0f / M_PI;

				return info;
			}

			void onNewFrame(openni::VideoStream& stream);

		private:

			bool initOpenNI2();
			void deinitOpenNI2();
			openni::Device device_;
			openni::VideoStream depthStream_;
			openni::VideoStream colorStream_;
			
			openni::Recorder recorder_;
			bool shouldRecord_;
			std::string outputFilePath_;

			//HoloCaptureOpenNI2Listener depthListener_;
			//HoloCaptureOpenNI2Listener colorListener_;

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
			float rgbHOV_;
			float rgbVOV_;
			float zHOV_;
			float zVOV_;
			bool isOpen_;
			bool isOpenNI2Init_;

			WorldConvertCache worldConvertCache_;

			std::string inputFilePath_;

			log4cxx::LoggerPtr logger_;
		};
	}
}