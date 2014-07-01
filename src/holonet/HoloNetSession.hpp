#pragma once
#include "../common/CommonDefs.hpp"
#include "HoloNet.hpp"
#include <boost/asio.hpp>
#include <thread>
#include <mutex>
#include <queue>
#include <condition_variable>



namespace holo
{
	namespace net
	{
		class HoloNetSession
		{
		public:
			HoloNetSession();
			~HoloNetSession();
			
			void sendPacket(boost::shared_ptr<HoloNetPacket> && packet);
			void recvPacket(boost::shared_ptr<HoloNetPacket> & packet);

			bool isConnected();

			void performHandshake(HoloNetProtocolHandshake localInfo);

			static HoloNetProtocolHandshake GetHandshakeFromPacket(boost::shared_ptr<HoloNetPacket> & packet);

		protected:
			//void start();

			//std::thread sendQueueThread_;
			//std::thread recvQueueThread_;
			boost::shared_ptr<boost::asio::ip::tcp::socket> socket_;
			//void sendLoop();
			//void recvLoop();


		private:

			//void popLocalPacket(boost::shared_ptr<HoloNetPacket> & packet);
			//void pushLocalPacket(boost::shared_ptr<HoloNetPacket> && packet);

			//void popRemotePacket(boost::shared_ptr<HoloNetPacket> & packet);
			//void pushRemotePacket(boost::shared_ptr<HoloNetPacket> && packet);

			//std::queue<boost::shared_ptr<HoloNetPacket>> sendQueue_;
			//std::queue<boost::shared_ptr<HoloNetPacket>> recvQueue_;
			//std::mutex sendQueueMutex_;
			//std::mutex recvQueueMutex_;

			//std::condition_variable haveLocalPacket_;
			//std::condition_variable haveNewRemotePacket_;

			log4cxx::LoggerPtr logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.net.session");
		};
	}
}
