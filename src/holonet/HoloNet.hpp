#pragma once
#include "../common/CommonDefs.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>


#define HOLO_NET_MAGIC_NUMBER 42685
#define HOLO_NET_DEFAULT_PORT 42685

#define HOLO_NET_CURRENT_PROTOCOL_VERSION 1
#define HOLO_NET_MAX_SUPPORTED_PROTOCOL_VERSION 1
#define HOLO_NET_NAME_STR_SIZE 256

namespace holo
{
	namespace net
	{
		enum HOLO_NET_PACKET_TYPE
		{
			HOLO_NET_PACKET_TYPE_UNKNOWN,
			HOLO_NET_PACKET_TYPE_HANDSHAKE,
			HOLO_NET_PACKET_TYPE_GRACEFUL_DISCONNECT,
			HOLO_NET_PACKET_TYPE_AUDIO_COMPRESSED,
			HOLO_NET_PACKET_TYPE_RGBZ_FRAME_COMPRESSED,
			HOLO_NET_PACKET_TYPE_OCTREE_COMPRESSED
		};

		struct HoloNetProtocolHandshake
		{
			uint32_t magicNumber;
			uint32_t protocolVersion;
			uint8_t clientName[HOLO_NET_NAME_STR_SIZE];
			int codecType;
			uint32_t rgbazWidth;
			uint32_t rgbazHeight;
			float captureFPS;
			float captureHOV;
			float captureVOV;
		};

		struct HoloNetPacket
		{
			uint32_t type;
			uint32_t length;
			std::vector<uint8_t> value;
		};
	}
}