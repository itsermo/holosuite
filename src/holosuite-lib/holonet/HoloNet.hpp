#pragma once
#include "../holocommon/CommonDefs.hpp"
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>

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
			HOLO_NET_PACKET_TYPE_OCTREE_COMPRESSED,
			HOLO_NET_PACKET_TYPE_OBJECT_ADD,
			HOLO_NET_PACKET_TYPE_OBJECT_DEL,
			HOLO_NET_PACKET_TYPE_OBJECT_UPDATE,
			HOLO_NET_PACKET_TYPE_OBJECT_CHANGE_OWNER,
		};

		struct HoloNetProtocolHandshake
		{
			uint32_t magicNumber;
			uint32_t protocolVersion;
			uint8_t clientName[HOLO_NET_NAME_STR_SIZE];
			int videoCodecType;
			int audioCodecType;
			uint32_t rgbazWidth;
			uint32_t rgbazHeight;
			float captureFPS;
			float captureHOV;
			float captureVOV;
			int audioNumChan;
			int audioFreq;
			int audioBitDepth;
		};

		class HoloNetPacket
		{
		public:
			HoloNetPacket(HOLO_NET_PACKET_TYPE packetType, uint32_t dataLen)
			{
				packetData_.resize(dataLen + sizeof(uint32_t)* 2);
				type_ = reinterpret_cast<uint32_t*>(packetData_.data());
				length_ = reinterpret_cast<uint32_t*>(packetData_.data() + sizeof(uint32_t));

				*type_ = boost::asio::detail::socket_ops::host_to_network_long(static_cast<uint32_t>(packetType));
				*length_ = boost::asio::detail::socket_ops::host_to_network_long(static_cast<uint32_t>(dataLen));

				isRemotePacket_ = false;
			};

			HoloNetPacket(std::vector<uint8_t> hostDataPacket)
			{
				isRemotePacket_ = true;
				packetData_.resize(hostDataPacket.size());
				std::copy(hostDataPacket.begin(), hostDataPacket.end(), packetData_.begin());
				type_ = reinterpret_cast<uint32_t*>(packetData_.data());
				length_ = reinterpret_cast<uint32_t*>(packetData_.data() + sizeof(uint32_t));
			}

			~HoloNetPacket()
			{

			}

			HOLO_NET_PACKET_TYPE GetPacketType()
			{
				if (!isRemotePacket_)
					return static_cast<HOLO_NET_PACKET_TYPE>(boost::asio::detail::socket_ops::host_to_network_long(*type_));
				else
					return static_cast<HOLO_NET_PACKET_TYPE>(boost::asio::detail::socket_ops::network_to_host_long(*type_));
			}

			uint32_t GetPacketLength()
			{
				if (!isRemotePacket_)
					return boost::asio::detail::socket_ops::host_to_network_long(*length_);
				else
					return boost::asio::detail::socket_ops::network_to_host_long(*length_);
			}

			uint8_t* GetPacketValue()
			{
				return (packetData_.data() + sizeof(uint32_t)* 2);
			}

			std::vector<uint8_t>& GetPacketBuffer()
			{
				return packetData_;
			}

		private:
			bool isRemotePacket_;
			uint32_t *type_;
			uint32_t *length_;
			std::vector<uint8_t> packetData_;
		};

		//struct HoloNetPacket
		//{
		//	uint32_t type;
		//	uint32_t length;
		//	std::vector<uint8_t> value;
		//};
	}
}