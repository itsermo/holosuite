#pragma once

#include "IHoloCodec.hpp"
#include <pcl/compression/octree_pointcloud_compression.h>
#include <log4cxx/logger.h>

#define HOLO_CODEC_OCTREE_DEFAULT_SHOW_STATISTICS false
#define HOLO_CODEC_OCTREE_DEFAULT_POINT_RESOLUTION 0.005f
#define HOLO_CODEC_OCTREE_DEFAULT_OCTREE_RESOLUTION 0.001f
#define HOLO_CODEC_OCTREE_DEFAULT_DO_VOXEL_GRID_DOWNSAMPLING true
#define HOLO_CODEC_OCTREE_DEFAULT_FRAMERATE 30
#define HOLO_CODEC_OCTREE_DEFAULT_DO_ENCODE_COLOR_INFO true
#define HOLO_CODEC_OCTREE_DEFAULT_COLOR_BIT_RESOLUTION (unsigned char)'\006'

namespace holo
{
	namespace codec
	{
		struct HoloCodecOctreeEncodeArgs
		{
			bool showStatistics;
			double pointResolution;
			double octreeResolution;
			bool doVoxelGridDownsampling;
			unsigned int frameRate;
			bool doEncodeColorInfo;
			unsigned char colorBitResolution;
		};

		class HoloCodecOctree : public IHoloCodec<HoloCloud>
		{

		public:
			HoloCodecOctree();
			HoloCodecOctree(HoloCodecOctreeEncodeArgs args);
			~HoloCodecOctree();

			bool init(CODEC_MODE codecMode_);
			void deinit();

			void encode(boost::shared_ptr<HoloCloud> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut);
			void decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut);

			CODEC_MODE getcodecMode() { return codecMode_; }

			bool isInit() { return isInit_; }

		private:
			bool isInit_;
			CODEC_MODE codecMode_;
			pcl::io::compression_Profiles_e compressionProfile_;
			boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>> encoder_;
			boost::shared_ptr<pcl::io::OctreePointCloudCompression<HoloPoint3D>> decoder_;
			HoloCodecOctreeEncodeArgs args_;

			log4cxx::LoggerPtr logger_;
		};
	}
}