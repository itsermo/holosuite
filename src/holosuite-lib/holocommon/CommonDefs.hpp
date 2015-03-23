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

	struct PointXYZW
	{
		union EIGEN_ALIGN16 {
			float data[4];
			struct {
				float x;
				float y;
				float z;
				float w;
			};
		};

		inline Eigen::Map<Eigen::Vector3f> getVector3fMap() { return (Eigen::Vector3f::Map(data)); }
		inline const Eigen::Map<const Eigen::Vector3f> getVector3fMap() const { return (Eigen::Vector3f::Map(data)); }
		inline Eigen::Map<Eigen::Vector4f, Eigen::Aligned> getVector4fMap() { return (Eigen::Vector4f::MapAligned(data)); }
		inline const Eigen::Map<const Eigen::Vector4f, Eigen::Aligned> getVector4fMap() const { return (Eigen::Vector4f::MapAligned(data)); }
		inline Eigen::Map<Eigen::Array3f> getArray3fMap() { return (Eigen::Array3f::Map(data)); }
		inline const Eigen::Map<const Eigen::Array3f> getArray3fMap() const { return (Eigen::Array3f::Map(data)); }
		inline Eigen::Map<Eigen::Array4f, Eigen::Aligned> getArray4fMap() { return (Eigen::Array4f::MapAligned(data)); }
		inline const Eigen::Map<const Eigen::Array4f, Eigen::Aligned> getArray4fMap() const { return (Eigen::Array4f::MapAligned(data)); }
	};

	typedef pcl::PointXYZ HoloVec3f;
	typedef PointXYZW HoloVec4f;
	typedef Eigen::Matrix4f HoloMat4f;
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

	namespace input
	{
		typedef enum
		{
			INPUT_DEVICE_TYPE_NONE = -1,
			INPUT_DEVICE_TYPE_LEAPSDK = 0
		} INPUT_DEVICE_TYPE;
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
			RENDER_TYPE_OPENGL = 2,
			RENDER_TYPE_DSCP_MKII = 3,
			RENDER_TYPE_DSCP_MKIV = 4
		} RENDER_TYPE;
		
		struct HoloObjectHeader
		{
			unsigned int num_vertices;
			unsigned int num_points_per_vertex;
			unsigned int num_color_channels;
			unsigned int vertex_stride;
			unsigned int color_stride;
			unsigned int num_indecies; // points, lines, triangles, or quads? can only be { 1,2,3,4 }
		};

		struct HoloTransform
		{
			//w is radius squared, xyz is center
			HoloVec4f bounding_sphere;

			//axis/angle, where w is angle
			HoloVec4f rotation;
			HoloVec3f scale, translate;
		};
		
	}

	namespace input
	{
		typedef enum
		{
			GESTURE_TYPE_NONE = 0,
			GESTURE_TYPE_CIRCLE = 1,
			GESTURE_TYPE_SWAP = 2,
			GESTURE_TYPE_KEY_TAP = 3,
			GESTURE_TYPE_SCREEN_TAP = 4
		} GESTURE_TYPE;

		struct HoloBone
		{
			HoloVec3f center, direction;
		};

		struct HoloFinger
		{
			HoloVec3f tipPosition, direction;
			HoloBone bones[4];
			bool isExtended;
		};

		struct HoloHand
		{
			HoloFinger thumb, index, middle, ring, pinky;
			HoloVec3f palmPosition, palmNormal;
			float grabStrength, pinchStrength;
			GESTURE_TYPE gesture;
		};

		struct HoloInputData
		{
			HoloHand leftHand, rightHand;
		};
	}
};