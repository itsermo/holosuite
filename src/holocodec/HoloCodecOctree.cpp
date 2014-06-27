#include "HoloCodecOctree.hpp"
#include <sstream>
#include <cstdint>
#include <chrono>

using namespace holo;
using namespace holo::codec;

HoloCodecOctree::HoloCodecOctree() : 
	isInit_(false),
	compressionProfile_((pcl::io::compression_Profiles_e)pcl::io::LOW_RES_ONLINE_COMPRESSION_WITH_COLOR),
	args_()
{
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
	LOG4CXX_DEBUG(logger_, "HoloCodecOctree object instantiated with custom args");

}

HoloCodecOctree::~HoloCodecOctree()
{
	if (isInit_)
		deinit();
	LOG4CXX_DEBUG(logger_, "HoloCodecOctree object destroyed");
}

bool HoloCodecOctree::init(CODEC_TYPE codecType)
{
	deinit();

	codecType_ = codecType;

	switch (codecType_)
	{
	case holo::codec::CODEC_TYPE_ENCODER:
		LOG4CXX_INFO(logger_, "Initializing PCL octree codec...");
		LOG4CXX_DEBUG(logger_, "Using custom args: \n"
			<< "Show statistics: " << args_.showStatistics << std::endl
			<< "Point resolution: " << args_.pointResolution << std::endl
			<< "Octree resolution: " << args_.octreeResolution << std::endl
			<< "Voxel grid downsampling: " << args_.doVoxelGridDownsampling << std::endl
			<< "Framerate: " << args_.frameRate << std::endl
			<< "Encode color info: " << args_.doEncodeColorInfo << std::endl
			<< "Color bit resolution: " << args_.colorBitResolution << std::endl);
		encoder_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>>(new pcl::io::OctreePointCloudCompression<HoloPoint3D>(compressionProfile_)
			);
		break;
	case holo::codec::CODEC_TYPE_DECODER:
		LOG4CXX_INFO(logger_, "Initializing PCL octree decoder only...");
		decoder_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>>(new pcl::io::OctreePointCloudCompression<HoloPoint3D>());
		LOG4CXX_INFO(logger_, "PCL octree decoder intialized");
		break;
	case holo::codec::CODEC_TYPE_BOTH:
	{
		decoder_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>>(new pcl::io::OctreePointCloudCompression<HoloPoint3D>());
		encoder_ = boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>>(new pcl::io::OctreePointCloudCompression<HoloPoint3D>(compressionProfile_));
	}
			break;
	default:
		LOG4CXX_ERROR(logger_, "Could not initialize PCL octree compression because codec type was set to invalid value");
		return false;
		break;
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

		auto t_start = std::chrono::high_resolution_clock::now();

		encoder_->encodePointCloud((HoloCloud::ConstPtr)rawData, compressedData);
		auto t_end = std::chrono::high_resolution_clock::now();
		std::cout << "Encode took " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << "ms" << std::endl;

		std::string compressedString = compressedData.str();
		encodeOut = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>(compressedString.begin(), compressedString.end()));
	}
}

void HoloCodecOctree::decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut)
{
	if (isInit_)
	{
		std::istringstream encodedStringStream(std::string(encodedStream->begin(), encodedStream->end()));
		decoder_->decodePointCloud(encodedStringStream, decodeOut);
	}
}
