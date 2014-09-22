#include "HoloRenderVisualizer.hpp"

using namespace holo;
using namespace holo::render;

HoloRenderVisualizer::HoloRenderVisualizer() : HoloRenderVisualizer(HOLO_RENDER_DEFAULT_VOXEL_SIZE, HOLO_CAPTURE_DEFAULT_Z_WIDTH, HOLO_CAPTURE_DEFAULT_Z_WIDTH)
{

}

HoloRenderVisualizer::HoloRenderVisualizer(int voxelSize, int zWidth, int zHeight) :
	voxelSize_(voxelSize),
	zWidth_(zWidth),
	zHeight_(zHeight),
	shouldRun_(false),
	haveNewCloud_(false),
	isInit_(false),
	enableMeshConstruction_(false),
	firstTime_(true)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.visualizer");

	LOG4CXX_DEBUG(logger_, "HoloRenderVisualizer object instantiated");
}

HoloRenderVisualizer::~HoloRenderVisualizer()
{
	deinit();
}

bool HoloRenderVisualizer::init()
{
	LOG4CXX_INFO(logger_, "Initializing point cloud visualizer...");

	if (enableMeshConstruction_)
	{
		organizedFastMesh_.setTrianglePixelSize(1);
		organizedFastMesh_.setTriangulationType(pcl::OrganizedFastMesh<HoloPoint3D>::TRIANGLE_ADAPTIVE_CUT);
	}

	cloudLock_ = std::unique_lock<std::mutex>(cloudMutex_);
	cloudLock_.unlock();

	shouldRun_ = true;


	auto initVisualizerLock = std::unique_lock<std::mutex>(initVisualizerMutex_);

	LOG4CXX_DEBUG(logger_, "Starting visualizer render thread, waiting for init notify...");
	renderThread_ = std::thread(&HoloRenderVisualizer::run, this);

	initVisualizerCV_.wait(initVisualizerLock);
	LOG4CXX_DEBUG(logger_, "Visualizer render thread init recieved");

	LOG4CXX_INFO(logger_, "Point cloud visualizier initialized");

	return isInit_;
}

void HoloRenderVisualizer::deinit()
{
	if (isInit_)
	{
		LOG4CXX_INFO(logger_, "Deinitializing point cloud visualizer...");
		shouldRun_ = false;
		renderThread_.join();
		isInit_ = false;
		LOG4CXX_INFO(logger_, "Point cloud visualizier deinitialized");
	}
}

void HoloRenderVisualizer::run()
{
	{
		std::lock_guard<std::mutex> lg(cloudMutex_);
		visualizer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("holosuite visualizer"));
		visualizer_->setBackgroundColor(0, 0, 0);

		//pointCloud_ = HoloCloudPtr(new HoloCloud(zWidth_, zHeight_));
		//pointCloud_->is_dense = false;
		//pointCloud_->sensor_origin_.setZero();
		//pointCloud_->sensor_orientation_.setIdentity();



		if (enableMeshConstruction_)
		{
			mesh_ = pcl::PolygonMeshPtr(new pcl::PolygonMesh());
			organizedFastMeshVertices_ = boost::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>);
		}

		//visualizer_->addCoordinateSystem(1.0, "global");
		visualizer_->initCameraParameters();

		isInit_ = true;
	}

	initVisualizerCV_.notify_one();

	LOG4CXX_DEBUG(logger_, "Point cloud visualizier render thread started");

	while (shouldRun_)
	{
		if (haveNewCloud_)
		{
			std::lock_guard<std::mutex> lg(cloudMutex_);
			if (enableMeshConstruction_)
			{
				if (!firstTime_)
					visualizer_->removePolygonMesh("holomesh");

				organizedFastMesh_.setInputCloud(pointCloud_);
				organizedFastMesh_.reconstruct(*organizedFastMeshVertices_);
				mesh_->polygons = *organizedFastMeshVertices_;
				pcl::PCLPointCloud2 cloud2;
				pcl::toPCLPointCloud2<HoloPoint3D>((const HoloCloud)*pointCloud_, cloud2);
				mesh_->cloud = cloud2;

				visualizer_->addPolygonMesh(*mesh_, "holomesh");
			}
			else
			{
				if (firstTime_)
				{
					visualizer_->addPointCloud(pointCloud_, "holocloud");
					visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, voxelSize_, "holocloud");
				}
				else
					visualizer_->updatePointCloud(pointCloud_, "holocloud");
			}

			firstTime_ = false;
			haveNewCloud_ = false;
		}

		visualizer_->spinOnce(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	LOG4CXX_DEBUG(logger_, "Point cloud visualizier render thread finished");
}

void HoloRenderVisualizer::updateFromPointCloud(HoloCloudPtr && pointCloud)
{
	cloudLock_.lock();
	pointCloud_ = std::move(pointCloud);
	haveNewCloud_ = true;
	cloudLock_.unlock();
}

void* HoloRenderVisualizer::getContext()
{
	return (void*)visualizer_.get();
}