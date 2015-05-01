#pragma once
#include "../holocommon/CommonDefs.hpp"
#include "HoloNet.hpp"
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

#ifdef ENABLE_HOLO_UDT
#include <udt.h>
#endif

namespace holo
{
	namespace net
	{
		class HoloNetSession
		{
		public:
			HoloNetSession();
			~HoloNetSession();
			
			void sendPacketAsync(boost::shared_ptr<HoloNetPacket> && packet);
			void sendPacket(boost::shared_ptr<HoloNetPacket> && packet);
			void recvPacket(boost::shared_ptr<HoloNetPacket> & packet);

			bool isConnected();

#ifdef ENABLE_HOLO_UDT
			void performHandshake(HoloNetProtocolHandshake localInfo, UDTSOCKET socket);
#else
			void performHandshake(HoloNetProtocolHandshake localInfo, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
#endif

			static HoloNetProtocolHandshake GetHandshakeFromPacket(boost::shared_ptr<HoloNetPacket> & packet);

			void disconnect();

		protected:
			void start();

#ifdef ENABLE_HOLO_UDT
			void addSocket(UDTSOCKET socket)
			{
				this->sockets_.push_back(socket);
			}

			std::vector<UDTSOCKET> sockets_;
#else
			void addSocket(boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
			{
				this->sockets_.push_back(socket);
			}

			std::vector<boost::shared_ptr<boost::asio::ip::tcp::socket>> sockets_;
#endif

			std::thread sendQueueThread_;


			void sendLoop();

			bool isConnected_;

#ifdef ENABLE_HOLO_UDT
			static void sendPacket(boost::shared_ptr<HoloNetPacket> & packet, UDTSOCKET socket);
			static void recvPacket(boost::shared_ptr<HoloNetPacket> & packet, UDTSOCKET socket);
#else
			static void sendPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
			static void recvPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
#endif

		private:

			void popLocalPacket(boost::shared_ptr<HoloNetPacket> & packet);
			void pushLocalPacket(boost::shared_ptr<HoloNetPacket> && packet);

			std::mutex socketListMutex_;

			std::queue<boost::shared_ptr<HoloNetPacket>> sendQueue_;

			std::mutex sendQueueMutex_;

			std::condition_variable haveLocalPacketCV_;
			std::atomic<bool> haveLocalPacket_;

			std::atomic<bool> shouldSend_;

			log4cxx::LoggerPtr logger_;
		};
	}
}
