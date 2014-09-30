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

#define HOLO_SESSION_CV_WAIT_TIMEOUT_MS 500

#define HOLO_RENDER_DEFAULT_VOXEL_SIZE 3

#define HOLO_CODEC_OCTREE_DEFAULT_PROFILE 1

#define HOLO_CAPTURE_RGBAZ_ARRAY_SIZE 2

#define HOLO_NET_PACKET_BUFFER_SIZE 10

#ifdef ENABLE_HOLO_AUDIO
#define HOLO_AUDIO_DEFAULT_FMT_FREQ 48000
#define HOLO_AUDIO_DEFAULT_FMT_CHAN 2
#define HOLO_AUDIO_DEFAULT_FMT_DEPTH 16
#define HOLO_AUDIO_DEFAULT_VOLUME_MIN 0.0f
#define HOLO_AUDIO_DEFAULT_VOLUME_MAX 1.0f
#define HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE 960
#define HOLO_AUDIO_DEFAULT_ENCODE_SIGNAL OPUS_AUTO
#define HOLO_AUDIO_DEFAULT_ENCODE_BITRATE 48000
#define HOLO_AUDIO_DEFAULT_ENCODE_BANDWIDTH OPUS_BANDWIDTH_FULLBAND
#define HOLO_AUDIO_DEFAULT_NUM_FRAMES HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE
#endif

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

	struct HoloAudioFormat
	{
		unsigned int frequency;
		unsigned int numChannels;
		unsigned int depth;
	};

	typedef enum
	{
		HOLO_SESSION_MODE_SERVER = 0,
		HOLO_SESSION_MODE_CLIENT = 1,
		HOLO_SESSION_MODE_LOOPBACK = 2,
		HOLO_SESSION_MODE_DIRECT = 3
	} HOLO_SESSION_MODE;

	namespace capture
	{
		typedef enum
		{
			CAPTURE_AUDIO_TYPE_NONE = -1,
			CAPTURE_AUDIO_TYPE_PORTAUDIO = 0
		} CAPTURE_AUDIO_TYPE;

		typedef enum
		{
			CAPTURE_TYPE_NONE = -1,
			CAPTURE_TYPE_FILE_PLY = 0,
			CAPTURE_TYPE_FILE_PCD = 1,
			CAPTURE_TYPE_FILE_OBJ = 2,
			CAPTURE_TYPE_FILE_ONI = 3,
			CAPTURE_TYPE_OPENNI2 = 4,
		} CAPTURE_TYPE;

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

		typedef enum
		{
			CODEC_TYPE_NONE = -1,
			CODEC_TYPE_PASSTHROUGH_CLOUD = 0,
			CODEC_TYPE_PASSTHROUGH_RGBAZ = 1,
			CODEC_TYPE_OCTREE = 2,
			CODEC_TYPE_H264 = 3,
			CODEC_TYPE_OPUS = 4
		} CODEC_TYPE;

		typedef enum
		{
			CODEC_MODE_ENCODER = 0,
			CODEC_MODE_DECODER = 1,
			CODEC_MODE_BOTH = 3
		} CODEC_MODE;

		typedef enum
		{
			CODEC_METHOD_PIXMAP = 0,
			CODEC_METHOD_CLOUD = 1,
			CODEC_METHOD_AUDIO = 2,
		}CODEC_METHOD ;
	}

	namespace render
	{
		typedef enum
		{
			RENDER_AUDIO_TYPE_NONE = -1,
			RENDER_AUDIO_TYPE_PORTAUDIO = 0
		}RENDER_AUDIO_TYPE;

		typedef enum
		{
			RENDER_TYPE_NONE = -1,
			RENDER_TYPE_VIS2D = 0,
			RENDER_TYPE_VIS3D = 1,
			RENDER_TYPE_DSCP_MKII = 2,
			RENDER_TYPE_DSCP_MKIV = 3
		} RENDER_TYPE;
	}
};