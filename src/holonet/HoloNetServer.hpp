#pragma once
#include "../common/CommonDefs.hpp"
#include "HoloNetSession.hpp"
#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <string>

namespace holo
{
	namespace net
	{
		class HoloNetServer : public HoloNetSession
		{
		public:
			HoloNetServer();
			~HoloNetServer();

			HoloNetProtocolHandshake listenAndWait(unsigned short port, HoloNetProtocolHandshake localInfo);
			//void disconnect();

		private:

			boost::shared_ptr<boost::asio::io_service> io_service_;
			boost::shared_ptr<boost::asio::ip::tcp::acceptor> acceptor_;

			log4cxx::LoggerPtr logger_;
		};
	}
}