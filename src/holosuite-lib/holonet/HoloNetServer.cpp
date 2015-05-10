#include "HoloNetServer.hpp"

using namespace holo;
using namespace holo::net;

HoloNetServer::HoloNetServer() : HoloNetSession(), shouldListen_(false)
{
	 logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.net.server");
}

HoloNetServer::~HoloNetServer()
{
	disconnect();
	stopListening();
}

void HoloNetServer::listenAsync(unsigned short port, HoloNetProtocolHandshake localInfo)
{
	port_ = port;
	localInfo_ = localInfo;

	if (!shouldListen_)
	{
		shouldListen_.store(true);
		listenThread_ = std::thread(&HoloNetServer::listenLoop, this);
	}
}

void HoloNetServer::stopListening()
{
	shouldListen_.store(false);

#ifdef ENABLE_HOLO_UDT

#else
	acceptor_->cancel();
	acceptor_->close();
#endif

	listenThread_.join();
}

HoloNetProtocolHandshake HoloNetServer::waitForNextClient()
{
	while (handShakes_.size() == 0)
	{
		std::this_thread::sleep_for(std::chrono::seconds(1));
	}

	std::unique_lock<std::mutex> hslock(handShakeMutex_);
	//haveNewHandShake_.wait(hslock);
	auto hs = handShakes_.front();
	handShakes_.pop_back();
	hslock.unlock();
	return hs;
}

void HoloNetServer::listenLoop()
{
	
#ifdef ENABLE_HOLO_UDT

	UDT::startup();

	addrinfo hints;
	addrinfo* res;

	memset(&hints, 0, sizeof(struct addrinfo));

	hints.ai_flags = AI_PASSIVE;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;

	std::string port(std::to_string(port_));

	if (0 != getaddrinfo(NULL, port.c_str(), &hints, &res))
	{
		LOG4CXX_ERROR(logger_, "Illegal UDT port number or port is busy.")
		return;
	}

	UDTSOCKET serv = UDT::socket(res->ai_family, res->ai_socktype, res->ai_protocol);
	int ret = UDT::setsockopt(serv, 0, UDT_MSS, new int(9000), sizeof(int));
	ret = UDT::setsockopt(serv, 0, UDT_RCVBUF, new int(10000000), sizeof(int));
	ret = UDT::setsockopt(serv, 0, UDP_RCVBUF, new int(10000000), sizeof(int));

	if (UDT::ERROR == UDT::bind(serv, res->ai_addr, res->ai_addrlen))
	{
		LOG4CXX_ERROR(logger_, "Could not bind UDT socket: " << UDT::getlasterror().getErrorMessage())
		return;
	}

	freeaddrinfo(res);

	LOG4CXX_INFO(logger_, "UDT server is ready at port: " << port_)

	if (UDT::ERROR == UDT::listen(serv, 10))
	{
		LOG4CXX_ERROR(logger_, "Could not listen: " << UDT::getlasterror().getErrorMessage())
		return;
	}

	sockaddr_storage clientaddr;
	int addrlen = sizeof(clientaddr);


#else
	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port_);

	io_service_ = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service);
	

	acceptor_ = boost::shared_ptr<boost::asio::ip::tcp::acceptor>(new boost::asio::ip::tcp::acceptor(*io_service_));

	acceptor_->open(endpoint.protocol());
	acceptor_->bind(endpoint);
	acceptor_->listen();
#endif

	while (shouldListen_)
	{

#ifdef ENABLE_HOLO_UDT
		auto socket = UDT::accept(serv, (sockaddr*)&clientaddr, &addrlen);
		if (socket == UDT::INVALID_SOCK)
		{
			LOG4CXX_ERROR(logger_, "Could not accept UDT socket")
			shouldListen_ = false;
			return;
		}

		//ret = UDT::setsockopt(socket, 0, UDT_RCVBUF, new int(10000000), sizeof(int));
		//ret = UDT::setsockopt(socket, 0, UDP_RCVBUF, new int(10000000), sizeof(int));
		//ret = UDT::setsockopt(socket, 0, UDT_SNDBUF, new int(10000000), sizeof(int));
		//ret = UDT::setsockopt(socket, 0, UDP_SNDBUF, new int(10000000), sizeof(int));
#else
		auto socket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(*io_service_));
		boost::system::error_code ec;
		acceptor_->accept(*socket, ec);

		if (ec.value() != boost::system::errc::success)
		{
			shouldListen_ = false;
			return;
		}

		socket->set_option(boost::asio::ip::tcp::no_delay(true));
		//socket_->set_option(boost::asio::socket_base::send_buffer_size(65536));
		//socket_->set_option(boost::asio::socket_base::receive_buffer_size(65536));
#endif
		LOG4CXX_DEBUG(logger_, "Accepted socket connection from a client on port " << port_);

		isConnected_ = true;

		this->performHandshake(localInfo_, socket);

		LOG4CXX_DEBUG(logger_, "Trying to get handshake info from client" << port_);
		boost::shared_ptr<HoloNetPacket> handshakePacket;
		recvPacket(handshakePacket, socket);

		auto hs = GetHandshakeFromPacket(handshakePacket);

		LOG4CXX_INFO(logger_, "Client " << hs.clientName << " connected.");
		LOG4CXX_INFO(logger_, "Magic number: " << hs.magicNumber);
		LOG4CXX_INFO(logger_, "Protocol version: " << hs.protocolVersion);
		LOG4CXX_INFO(logger_, "RGBAZ Mode: " << hs.rgbazWidth << "x" << hs.rgbazHeight << "@" << hs.captureFPS);
		LOG4CXX_INFO(logger_, "RGBAZ Field-of-view: " << hs.captureHOV << " deg horiz, " << hs.captureVOV << " deg vert");
		LOG4CXX_INFO(logger_, "Audio Mode: " << hs.audioNumChan << " chan, " << hs.audioBitDepth << " bits @ " << hs.audioFreq << " kHz");

		this->addSocket(socket);

		std::unique_lock<std::mutex> hsLock(handShakeMutex_);
		handShakes_.push_back(hs);
		hsLock.unlock();
		haveNewHandShake_.notify_all();

		start();
	}

#ifdef ENABLE_HOLO_UDT
	UDT::close(serv);
	UDT::cleanup();
#endif

}
