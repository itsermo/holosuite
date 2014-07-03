#pragma once

#include "../common/CommonDefs.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace holo
{
	namespace render
	{
		class IHoloRender
		{
		public:
			virtual ~IHoloRender() = 0;
			virtual bool init() = 0;
			virtual void deinit() = 0;
			//virtual void updateFromMats(cv::Mat rgbaImage, cv::Mat depthImage) = 0;
			virtual void updateFromPointCloud(HoloCloudPtr && pointCloud) = 0;
			virtual void* getContext() = 0;
		};

		inline IHoloRender::~IHoloRender() { }
	}
}