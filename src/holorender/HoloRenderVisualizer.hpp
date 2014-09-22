#pragma once

#include "../common/CommonDefs.hpp"
#include "IHoloRender.hpp"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>

#define HOLO_RENDER_VISUALIZER_DEFAULT_VOXEL_SIZE HOLO_RENDER_DEFAULT_VOXEL_SIZE
#define HOLO_RENDER_VISUALIZER_DEFAULT_ENABLE_MESH_CONSTRUCTION false

namespace holo
{
	namespace render
	{
		class HoloRenderVisualizer : public IHoloRender
		{
		public:
			HoloRenderVisualizer();
			HoloRenderVisualizer(int voxelSize, bool enableMeshContruction);
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

			pcl::OrganizedFastMesh<HoloPoint3D> organizedFastMesh_;
			boost::shared_ptr<std::vector<pcl::Vertices>> organizedFastMeshVertices_;
			pcl::PolygonMesh::Ptr mesh_;

			HoloCloudPtr pointCloud_;
			std::thread renderThread_;
			std::unique_lock<std::mutex> cloudLock_;
			std::mutex cloudMutex_;

			std::mutex initVisualizerMutex_;
			std::condition_variable initVisualizerCV_;

			int zWidth_;
			int zHeight_;

			bool isInit_;

			bool firstTime_;

			bool enableMeshConstruction_;

			log4cxx::LoggerPtr logger_;
		};
	}
}