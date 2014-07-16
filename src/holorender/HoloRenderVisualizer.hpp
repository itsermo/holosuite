#pragma once

#include "../common/CommonDefs.hpp"
#include "IHoloRender.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

namespace holo
{
	namespace render
	{
		class HoloRenderVisualizer : public IHoloRender
		{
		public:
			HoloRenderVisualizer();
			HoloRenderVisualizer(int voxelSize, int zWidth, int zHeight);
			~HoloRenderVisualizer();
			virtual bool init();
			virtual void deinit();
			//virtual void updateFromMats(cv::Mat rgbaImage, cv::Mat depthImage) = 0;
			virtual void updateFromPointCloud(HoloCloudPtr && pointCloud);
			virtual void* getContext();
		
		private:

			void initVisualizer();
			void run();

			std::atomic<bool> shouldRun_;
			std::atomic<bool> haveNewCloud_;

			int voxelSize_;
			pcl::visualization::PCLVisualizer::Ptr visualizer_;
			HoloCloudPtr pointCloud_;
			std::thread renderThread_;
			std::unique_lock<std::mutex> cloudLock_;
			std::mutex cloudMutex_;

			std::mutex initVisualizerMutex_;
			std::condition_variable initVisualizerCV_;

			int zWidth_;
			int zHeight_;

			bool isInit_;

			log4cxx::LoggerPtr logger_;
		};
	}
}