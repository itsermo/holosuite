#include "HoloNetClient.hpp"

#include <string>

using namespace holo::net;

HoloNetClient::HoloNetClient() : HoloNetSession()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.net.client");

	LOG4CXX_TRACE(logger_, "Instantiating HoloClient object");

#ifndef ENABLE_HOLO_UDT
	resolver_ = boost::shared_ptr<boost::asio::ip::tcp::resolver>(new boost::asio::ip::tcp::resolver(io_service_));
#else
	UDT::startup();
#endif
}

HoloNetClient::~HoloNetClient()
{
	LOG4CXX_TRACE(logger_, "Destroying HoloClient object");
	disconnect();

#ifdef ENABLE_HOLO_UDT
	UDT::cleanup();
#endif
}

HoloNetProtocolHandshake HoloNetClient::connect(std::string address, int port, HoloNetProtocolHandshake localInfo)
{
	disconnect();

	LOG4CXX_INFO(logger_, "Resolving address " << address << ":" << port);

#ifdef ENABLE_HOLO_UDT
	struct addrinfo hints, *local, *peer;

	memset(&hints, 0, sizeof(struct addrinfo));

	hints.ai_flags = AI_PASSIVE;
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_DGRAM;
	//hints.ai_socktype = SOCK_DGRAM;

	std::string portstr(std::to_string(port));

	if (0 != getaddrinfo(NULL, portstr.c_str(), &hints, &local))
	{
		LOG4CXX_ERROR(logger_, "Incorrect network address")
		throw std::exception();
	}

	UDTSOCKET socket = UDT::socket(local->ai_family, local->ai_socktype, local->ai_protocol);

	//int ret = UDT::setsockopt(socket, 0, UDT_MSS, new int(9000), sizeof(int));
	//ret = UDT::setsockopt(socket, 0, UDT_RCVBUF, new int(10000000), sizeof(int));
	//ret = UDT::setsockopt(socket, 0, UDP_RCVBUF, new int(10000000), sizeof(int));
	//ret = UDT::setsockopt(socket, 0, UDT_SNDBUF, new int(10000000), sizeof(int));
	//ret = UDT::setsockopt(socket, 0, UDP_SNDBUF, new int(10000000), sizeof(int));

	freeaddrinfo(local);

	if (0 != getaddrinfo(address.c_str(), portstr.c_str(), &hints, &peer))
	{
		LOG4CXX_ERROR(logger_, "Incorrect server/peer address " << address << ":" << port)
		throw std::exception();
	}

	//std::this_thread::sleep_for(std::chrono::seconds(5));

	if (UDT::ERROR == UDT::connect(socket, peer->ai_addr, peer->ai_addrlen))
	{
		LOG4CXX_ERROR(logger_, "Connecting UDT socket: " << UDT::getlasterror().getErrorMessage())
		throw std::exception();
	}

	freeaddrinfo(peer);

#else
	auto socket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(io_service_));

	boost::asio::ip::tcp::resolver::query query(address.c_str(), std::to_string(port));
	boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver_->resolve(query);
	boost::asio::ip::tcp::resolver::iterator end;

	boost::system::error_code error = boost::asio::error::host_not_found;

	while (error && endpoint_iterator != end)
	{
		socket->close();
		socket->connect(*endpoint_iterator++, error);
	}
	if (error)
	{
		LOG4CXX_ERROR(logger_, "Could not connect to " << address << ":" << port << " Message: " << error.message() );
		throw boost::system::system_error(error);
	}

	socket->set_option(boost::asio::ip::tcp::no_delay(true), error);
	if (error)
	{
		LOG4CXX_WARN(logger_, "Could not set socket no delay option")
	}
	socket->set_option(boost::asio::socket_base::send_buffer_size(65536));
	socket->set_option(boost::asio::socket_base::receive_buffer_size(65536));

#endif

	isConnected_ = true;
	LOG4CXX_INFO(logger_, "Connected to " << address << ":" << port);

	performHandshake(localInfo, socket);

	boost::shared_ptr<HoloNetPacket> handshakePacket;
	this->recvPacket(handshakePacket, socket);

	auto hs = GetHandshakeFromPacket(handshakePacket);

	LOG4CXX_INFO(logger_, "Server name " << hs.clientName);
	LOG4CXX_INFO(logger_, "Magic number: " << hs.magicNumber);
	LOG4CXX_INFO(logger_, "Protocol version: " << hs.protocolVersion);
	LOG4CXX_INFO(logger_, "RGBAZ Mode: " << hs.rgbazWidth << "x" << hs.rgbazHeight << "@" << hs.captureFPS);
	LOG4CXX_INFO(logger_, "RGBAZ Field-of-view: " << hs.captureHOV << " deg horiz, " << hs.captureVOV << " deg vert");
	LOG4CXX_INFO(logger_, "Audio Mode: " << hs.audioNumChan << " chan, " << hs.audioBitDepth << " bits @ " << hs.audioFreq << " kHz");

	this->addSocket(socket);

	isConnected_ = true;

	start();

	return hs;
}



