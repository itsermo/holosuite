#pragma once

#include "../holocommon/CommonDefs.hpp"
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "HoloRenderObjectTracker.hpp"

namespace holo
{
	namespace render
	{
		class IHoloRender
		{
		public:
			virtual ~IHoloRender() = 0;
			virtual bool init(bool enableMirrorVisualFeedback = false) = 0;
			virtual void deinit() = 0;

			//virtual void updateFromMats(cv::Mat rgbaImage, cv::Mat depthImage) = 0;
			virtual void updateLocalPointCloud(HoloCloudPtr && pointCloud) = 0;
			virtual void updateRemotePointCloud(HoloCloudPtr && pointCloud) = 0;
			virtual void* getContext() = 0;

			virtual void addObjectTracker(boost::shared_ptr<HoloRenderObjectTracker> & objectTracker) = 0;
			virtual void removeObjectTracker() = 0;
		};

		inline IHoloRender::~IHoloRender() { }

	}
}