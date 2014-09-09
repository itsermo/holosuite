#pragma once
#include "../common/CommonDefs.hpp"
#include "HoloNetSession.hpp"
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <condition_variable>

namespace holo
{
	namespace net
	{
		class HoloNetServer : public HoloNetSession
		{
		public:
			HoloNetServer();
			~HoloNetServer();

			void listenAsync(unsigned short port, HoloNetProtocolHandshake localInfo);
			void stopListening();

			HoloNetProtocolHandshake waitForNextClient();
			//void disconnect();

		private:

			HoloNetProtocolHandshake localInfo_;
			unsigned short port_;
			std::vector<HoloNetProtocolHandshake> handShakes_;

			std::condition_variable haveNewHandShake_;
			std::mutex handShakeMutex_;

			std::atomic<bool> shouldListen_;
			std::thread listenThread_;
			void listenLoop();

			boost::shared_ptr<boost::asio::io_service> io_service_;
			boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;

			log4cxx::LoggerPtr logger_;
		};
	}
}