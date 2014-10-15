#pragma once

#include "../holocommon/CommonDefs.hpp"
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
			virtual void updateRemotePointCloud(HoloCloudPtr && pointCloud);
			virtual void updateLocalPointCloud(HoloCloudPtr && pointCloud);
			virtual void* getContext();
		
		private:

			void initVisualizer();
			void run();

			std::atomic<bool> shouldRun_;
			std::atomic<bool> haveNewRemoteCloud_;
			std::atomic<bool> haveNewLocalCloud_;

			int voxelSize_;
			pcl::visualization::PCLVisualizer::Ptr visualizer_;

			pcl::OrganizedFastMesh<HoloPoint3D> organizedFastMesh_;
			boost::shared_ptr<std::vector<pcl::Vertices>> organizedFastMeshVertices_;
			pcl::PolygonMesh::Ptr mesh_;

			HoloCloudPtr localCloud_;
			HoloCloudPtr remoteCloud_;

			std::thread renderThread_;

			std::unique_lock<std::mutex> remoteCloudLock_;
			std::mutex remoteCloudMutex_;

			std::unique_lock<std::mutex> localCloudLock_;
			std::mutex localCloudMutex_;

			std::mutex initVisualizerMutex_;
			std::condition_variable initVisualizerCV_;

			int zWidth_;
			int zHeight_;

			bool isInit_;

			bool firstTime_;

			bool enableMeshConstruction_;

			bool renderLocalPointCloud_;

			log4cxx::LoggerPtr logger_;
		};
	}
}