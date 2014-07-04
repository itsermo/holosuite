#include "HoloCodecOctree.hpp"
#include <sstream>
#include <cstdint>

#ifdef TRACE_LOG_ENABLED
#include <ctime>
#include <chrono>
#endif

using namespace holo;
using namespace holo::codec;

HoloCodecOctree::HoloCodecOctree() : 
	isInit_(false),
	compressionProfile_((pcl::io::compression_Profiles_e)pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR),
	args_()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.octree");

	args_.showStatistics = HOLO_CODEC_OCTREE_DEFAULT_SHOW_STATISTICS;
	args_.pointResolution = HOLO_CODEC_OCTREE_DEFAULT_POINT_RESOLUTION;
	args_.octreeResolution = HOLO_CODEC_OCTREE_DEFAULT_OCTREE_RESOLUTION;
	args_.doVoxelGridDownsampling = HOLO_CODEC_OCTREE_DEFAULT_DO_VOXEL_GRID_DOWNSAMPLING;
	args_.frameRate = HOLO_CODEC_OCTREE_DEFAULT_FRAMERATE;
	args_.doEncodeColorInfo = HOLO_CODEC_OCTREE_DEFAULT_DO_ENCODE_COLOR_INFO;
	args_.colorBitResolution = HOLO_CODEC_OCTREE_DEFAULT_COLOR_BIT_RESOLUTION;

	LOG4CXX_DEBUG(logger_, "HoloCodecOctree object instantiated with default args");

}

HoloCodecOctree::HoloCodecOctree(HoloCodecOctreeEncodeArgs args) :
	isInit_(false),
	compressionProfile_((pcl::io::compression_Profiles_e)pcl::io::MANUAL_CONFIGURATION),
	args_(args)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.octree");

	LOG4CXX_DEBUG(logger_, "HoloCodecOctree object instantiated with custom args");
}

HoloCodecOctree::~HoloCodecOctree()
{

	deinit();
	LOG4CXX_DEBUG(logger_, "HoloCodecOctree object destroyed");
}

bool HoloCodecOctree::init(CODEC_MODE codecMode)
{
	deinit();

	codecMode_ = codecMode;

	if (codecMode_ == CODEC_MODE_ENCODER || codecMode_ == CODEC_MODE_BOTH)
	{
		LOG4CXX_INFO(logger_, "Initializing PCL octree codec...");
		LOG4CXX_DEBUG(logger_, "Using custom args: \n"
			<< "Show statistics: " << args_.showStatistics << std::endl
			<< "Point resolution: " << args_.pointResolution << std::endl
			<< "Octree resolution: " << args_.octreeResolution << std::endl
			<< "Voxel grid downsampling: " << args_.doVoxelGridDownsampling << std::endl
			<< "Framerate: " << args_.frameRate << std::endl
			<< "Encode color info: " << args_.doEncodeColorInfo << std::endl
			<< "Color bit resolution: " << args_.colorBitResolution << std::endl);
		encoder_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>>(new pcl::io::OctreePointCloudCompression<HoloPoint3D>(compressionProfile_,
			args_.showStatistics,
			args_.pointResolution,
			args_.octreeResolution,
			args_.doVoxelGridDownsampling,
			args_.frameRate,
			args_.doEncodeColorInfo,
			args_.colorBitResolution
			)
			);
	}

	if (codecMode_ == CODEC_MODE_DECODER || codecMode_ == CODEC_MODE_BOTH)
	{
		LOG4CXX_INFO(logger_, "Initializing PCL octree decoder only...");
		decoder_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>>(new pcl::io::OctreePointCloudCompression<HoloPoint3D>());
		LOG4CXX_INFO(logger_, "PCL octree decoder intialized");
	}

	isInit_ = true;

	return true;
}

void HoloCodecOctree::deinit()
{
	if (isInit_)
	{
		isInit_ = false;
		encoder_.reset();
		decoder_.reset();
		LOG4CXX_INFO(logger_, "PCL octree codec deinitialized");
	}
}

void HoloCodecOctree::encode(boost::shared_ptr<HoloCloud> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut)
{
	std::clock_t c_start = std::clock();

	if (isInit_)
	{
		std::stringstream compressedData;

#ifdef TRACE_LOG_ENABLED
		auto startTime = std::chrono::system_clock::now();
#endif

		encoder_->encodePointCloud((HoloCloud::ConstPtr)rawData, compressedData);

#ifdef TRACE_LOG_ENABLED
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
		LOG4CXX_TRACE(logger_, "octree encoded a " << compressedData.tellp() << " byte compressed cloud frame in " << duration.count() << "ms");
#endif

		std::string compressedString = compressedData.str();
		encodeOut = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>(compressedString.begin(), compressedString.end()));
	}
}

void HoloCodecOctree::decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut)
{
	if (isInit_)
	{
		std::istringstream encodedStringStream(std::string(encodedStream->begin(), encodedStream->end()));
#ifdef TRACE_LOG_ENABLED
		auto startTime = std::chrono::system_clock::now();
#endif
		decoder_->decodePointCloud(encodedStringStream, decodeOut);
#ifdef TRACE_LOG_ENABLED
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
		LOG4CXX_TRACE(logger_, "octree decoded a " << encodedStream->size() << " byte compressed cloud frame in " << duration.count() << "ms");
#endif
	}
}
