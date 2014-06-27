#pragma once

#include <log4cxx/logger.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define HOLO_CAPTURE_DEFAULT_RGB_WIDTH 640
#define HOLO_CAPTURE_DEFAULT_RGB_HEIGHT 480
#define HOLO_CAPTURE_DEFAULT_RGB_FPS 30.0f
#define HOLO_CAPTURE_DEFAULT_Z_WIDTH 640
#define HOLO_CAPTURE_DEFAULT_Z_HEIGHT 480
#define HOLO_CAPTURE_DEFAULT_Z_FPS 30.0f

#define HOLO_RENDER_DEFAULT_VOXEL_SIZE 3

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

	namespace capture
	{
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
			CODEC_TYPE_ENCODER = 0,
			CODEC_TYPE_DECODER = 1,
			CODEC_TYPE_BOTH = 3
		};

		enum CODEC_METHOD
		{
			CODEC_METHOD_PIXMAP = 0,
			CODEC_METHOD_CLOUD = 1
		};
	}
};