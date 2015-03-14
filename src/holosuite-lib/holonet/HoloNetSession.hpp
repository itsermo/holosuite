#pragma once
#include "../holocommon/CommonDefs.hpp"
#include "HoloNet.hpp"
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>
#include <atomic>

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

			void performHandshake(HoloNetProtocolHandshake localInfo, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);

			static HoloNetProtocolHandshake GetHandshakeFromPacket(boost::shared_ptr<HoloNetPacket> & packet);

			void disconnect();

		protected:
			void start();

			void addSocket(boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
			{
				//std::lock_guard<std::mutex> lg(socketListMutex_);
				this->sockets_.push_back(socket);
			}

			std::thread sendQueueThread_;
			//std::thread recvQueueThread_;
			std::vector<boost::shared_ptr<boost::asio::ip::tcp::socket>> sockets_;
			void sendLoop();
			//void recvLoop();

			bool isConnected_;

			static void sendPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);
			static void recvPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket);

		private:

			void popLocalPacket(boost::shared_ptr<HoloNetPacket> & packet);
			void pushLocalPacket(boost::shared_ptr<HoloNetPacket> && packet);

			std::mutex socketListMutex_;

			//void popRemotePacket(boost::shared_ptr<HoloNetPacket> & packet);
			//void pushRemotePacket(boost::shared_ptr<HoloNetPacket> && packet);

			std::queue<boost::shared_ptr<HoloNetPacket>> sendQueue_;
			//std::queue<boost::shared_ptr<HoloNetPacket>> recvQueue_;
			std::mutex sendQueueMutex_;
			//std::mutex recvQueueMutex_;

			std::condition_variable haveLocalPacketCV_;
			std::atomic<bool> haveLocalPacket_;
			//std::condition_variable haveNewRemotePacket_;

			std::atomic<bool> shouldSend_;

			log4cxx::LoggerPtr logger_;
		};
	}
}
