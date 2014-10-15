#include "HoloRenderVisualizer.hpp"

using namespace holo;
using namespace holo::render;

HoloRenderVisualizer::HoloRenderVisualizer() : HoloRenderVisualizer(HOLO_RENDER_VISUALIZER_DEFAULT_VOXEL_SIZE, HOLO_RENDER_VISUALIZER_DEFAULT_ENABLE_MESH_CONSTRUCTION)
{

}

HoloRenderVisualizer::HoloRenderVisualizer(int voxelSize, bool enableMeshConstruction) : IHoloRender(),
	voxelSize_(voxelSize),
	shouldRun_(false),
	haveNewRemoteCloud_(false),
	haveNewLocalCloud_(false),
	isInit_(false),
	enableMeshConstruction_(enableMeshConstruction),
	firstTime_(true),
	renderLocalPointCloud_(true)
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

	remoteCloudLock_ = std::unique_lock<std::mutex>(remoteCloudMutex_);
	localCloudLock_ = std::unique_lock<std::mutex>(localCloudMutex_);
	remoteCloudLock_.unlock();
	localCloudLock_.unlock();

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
		std::lock_guard<std::mutex> rlg(remoteCloudMutex_);
		std::lock_guard<std::mutex> llg(localCloudMutex_);
		visualizer_ = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("holosuite visualizer"));
		visualizer_->setBackgroundColor(0, 0, 0);

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

	pcl::PointXYZ Pt;
	Pt.x = 0.15;
	Pt.y = 0;
	Pt.z = 0.6;
	visualizer_->addSphere(Pt, 0.1, 1.0, 0.0, 0.0, "Sphere0");
	
	//pcl::ModelCoefficients coeffs;
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(0.0);
	//coeffs.values.push_back(1.0);
	//coeffs.values.push_back(0.0);
	//visualizer_->addPlane(coeffs, 0.0, 1.0, 0.0, "plane");

	HoloCloudPtr planeCloud = HoloCloudPtr(new HoloCloud(200, 200));
	planeCloud->is_dense = false;
	planeCloud->sensor_origin_.setZero();
	planeCloud->sensor_orientation_.setIdentity();

	for (size_t y = 0, idx = 0; y < planeCloud->height; y++)
	{
		for (size_t x = 0; x < planeCloud->width; x++, idx++)
		{
			planeCloud->points[idx].x = (x / 200.0f) - 0.5;
			planeCloud->points[idx].y = (y/ 200.0f) - 0.5;
			planeCloud->points[idx].z = 0.7;
			planeCloud->points[idx].r = 0;
			planeCloud->points[idx].g = 255;
			planeCloud->points[idx].b = 255;
		}
	}

	visualizer_->addPointCloud(planeCloud, "planeCloud");

	while (shouldRun_)
	{
		if (haveNewRemoteCloud_)
		{
			std::lock_guard<std::mutex> lg(remoteCloudMutex_);
			if (enableMeshConstruction_)
			{
				if (!firstTime_)
					visualizer_->removePolygonMesh("holomesh");

				organizedFastMesh_.setInputCloud(remoteCloud_);
				organizedFastMesh_.reconstruct(*organizedFastMeshVertices_);
				mesh_->polygons = *organizedFastMeshVertices_;
				pcl::PCLPointCloud2 cloud2;
				pcl::toPCLPointCloud2<HoloPoint3D>((const HoloCloud)*remoteCloud_, cloud2);
				mesh_->cloud = cloud2;

				visualizer_->addPolygonMesh(*mesh_, "holomesh");
			}
			else
			{
				if (firstTime_)
				{
					visualizer_->addPointCloud(remoteCloud_, "holocloud");
					visualizer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, voxelSize_, "holocloud");
				}
				else
					visualizer_->updatePointCloud(remoteCloud_, "holocloud");
			}



			firstTime_ = false;
			haveNewRemoteCloud_ = false;
		}

		visualizer_->spinOnce(1);
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}

	LOG4CXX_DEBUG(logger_, "Point cloud visualizier render thread finished");
}

void HoloRenderVisualizer::updateRemotePointCloud(HoloCloudPtr && pointCloud)
{
	remoteCloudLock_.lock();
	remoteCloud_ = std::move(pointCloud);
	haveNewRemoteCloud_ = true;
	remoteCloudLock_.unlock();
}

void HoloRenderVisualizer::updateLocalPointCloud(HoloCloudPtr && pointCloud)
{
	localCloudLock_.lock();
	localCloud_ = std::move(pointCloud);
	haveNewLocalCloud_ = true;
	localCloudLock_.unlock();
}

void* HoloRenderVisualizer::getContext()
{
	return (void*)visualizer_.get();
}