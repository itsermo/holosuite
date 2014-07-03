#pragma once

#include "../common/CommonDefs.hpp"
#include "HoloCaptureInfo.hpp"
#include <list>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace holo
{
	namespace capture
	{
		class IHoloCapture
		{
		public:
			virtual ~IHoloCapture() = 0;
			virtual bool init(int which) = 0;
			virtual bool isOpen() = 0;
			virtual void waitAndGetNextFrame(cv::Mat& rgbaImage, cv::Mat& zImage) = 0;
			virtual void waitAndGetNextPointCloud(HoloCloudPtr& pointCloud) = 0;
			virtual void deinit() = 0;
			virtual std::list<std::string> enumerateDevices() = 0;
			virtual HoloCaptureInfo getCaptureInfo() = 0;
		};

		inline IHoloCapture::~IHoloCapture() { }
	};
};
