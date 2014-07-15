#pragma once

#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HOLO_CAPTURE_DEFAULT_RGB_WIDTH 320
#define HOLO_CAPTURE_DEFAULT_RGB_HEIGHT 240
#define HOLO_CAPTURE_DEFAULT_RGB_FPS 30.0f
#define HOLO_CAPTURE_DEFAULT_Z_WIDTH 320
#define HOLO_CAPTURE_DEFAULT_Z_HEIGHT 240
#define HOLO_CAPTURE_DEFAULT_Z_FPS 30.0f

#define HOLO_RENDER_DEFAULT_VOXEL_SIZE 5

#define HOLO_CODEC_OCTREE_DEFAULT_PROFILE 1

#define HOLO_CAPTURE_RGBAZ_ARRAY_SIZE 2

#define HOLO_NET_PACKET_BUFFER_SIZE 100

const float HOLO_CLOUD_BAD_POINT = std::numeric_limits<float>().quiet_NaN();

namespace holo
{
	typedef pcl::PointXYZRGBA HoloPoint3D;
	typedef pcl::PointCloud<HoloPoint3D> HoloCloud;
	typedef HoloCloud::Ptr HoloCloudPtr;
	struct HoloRGBAZMat { 
		cv::Mat rgba;
		cv::Mat z;
	};

	enum HOLO_SESSION_MODE
	{
		HOLO_SESSION_MODE_SERVER = 0,
		HOLO_SESSION_MODE_CLIENT = 1,
		HOLO_SESSION_MODE_FEEDBACK = 2,
		HOLO_SESSION_MODE_DIRECT = 3
	};

	namespace capture
	{
		
		enum CAPTURE_TYPE
		{
			CAPTURE_TYPE_FILE_PLY = 0,
			CAPTURE_TYPE_FILE_PCD = 1,
			CAPTURE_TYPE_FILE_OBJ = 2,
			CAPTURE_TYPE_FILE_ONI = 3,
			CAPTURE_TYPE_OPENNI2 = 4
		};

		struct WorldConvertCache
		{
			float xzFactor;
			float yzFactor;
			float resolutionX;
			float resolutionY;
			float halfResX;
			float halfResY;
			float coeffX;
			float coeffY;
		};
	}

	namespace codec
	{
		enum CODEC_TYPE
		{
			CODEC_TYPE_PASSTHROUGH_CLOUD,
			CODEC_TYPE_PASSTHROUGH_RGBAZ,
			CODEC_TYPE_OCTREE,
			CODEC_TYPE_H264
		};

		enum CODEC_MODE
		{
			CODEC_MODE_ENCODER = 0,
			CODEC_MODE_DECODER = 1,
			CODEC_MODE_BOTH = 3
		};

		enum CODEC_METHOD
		{
			CODEC_METHOD_PIXMAP = 0,
			CODEC_METHOD_CLOUD = 1
		};
	}

	namespace render
	{
		enum RENDER_TYPE
		{
			RENDER_TYPE_VIS2D = 0,
			RENDER_TYPE_VIS3D = 1,
			RENDER_TYPE_DSCP_MKII = 2,
			RENDER_TYPE_DSCP_MKIV = 3
		};
	}
};