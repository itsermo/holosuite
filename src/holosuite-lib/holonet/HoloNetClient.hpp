#pragma once

#include "HoloNet.hpp"
#include <string>
#include <thread>
#include <boost/asio.hpp>
#include <log4cxx/logger.h>
#include "HoloNetSession.hpp"

namespace holo
{
	namespace net
	{
		class HoloNetClient : public HoloNetSession
		{
		public:
			HoloNetClient();
			~HoloNetClient();

			HoloNetProtocolHandshake connect(std::string address, int port, HoloNetProtocolHandshake localInfo);
			//void disconnect();

		private:

#ifdef ENABLE_HOLO_UDT
			boost::shared_ptr<boost::asio::ip::udp::resolver> resolver_;
#else
			boost::shared_ptr<boost::asio::ip::tcp::resolver> resolver_;
			boost::asio::io_service io_service_;
#endif
			log4cxx::LoggerPtr logger_;
		};
	}
}