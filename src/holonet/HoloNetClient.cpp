#include "HoloNetClient.hpp"

#include <string>

using namespace holo::net;

HoloNetClient::HoloNetClient() : HoloNetSession()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.net.client");

	LOG4CXX_TRACE(logger_, "Instantiating HoloClient object");
	resolver_ = boost::shared_ptr<boost::asio::ip::tcp::resolver>(new boost::asio::ip::tcp::resolver(io_service_));
}

HoloNetClient::~HoloNetClient()
{
	LOG4CXX_TRACE(logger_, "Destroying HoloClient object");
	disconnect();
}

HoloNetProtocolHandshake HoloNetClient::connect(std::string address, int port, HoloNetProtocolHandshake localInfo)
{
	disconnect();

	LOG4CXX_INFO(logger_, "Resolving address " << address << ":" << port);

	socket_ = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(io_service_));

	boost::asio::ip::tcp::resolver::query query(address.c_str(), std::to_string(port));
	boost::asio::ip::tcp::resolver::iterator endpoint_iterator = resolver_->resolve(query);
	boost::asio::ip::tcp::resolver::iterator end;

	boost::system::error_code error = boost::asio::error::host_not_found;

	while (error && endpoint_iterator != end)
	{
		socket_->close();
		socket_->connect(*endpoint_iterator++, error);
	}
	if (error)
	{
		LOG4CXX_ERROR(logger_, "Could not connect to " << address << ":" << port << " Message: " << error.message() );
		throw boost::system::system_error(error);
	}

	socket_->set_option(boost::asio::ip::tcp::no_delay(true));
	//socket_->set_option(boost::asio::socket_base::send_buffer_size(65536));
	//socket_->set_option(boost::asio::socket_base::receive_buffer_size(65536));
	
	LOG4CXX_INFO(logger_, "Connected to " << address << ":" << port);

	performHandshake(localInfo);

	boost::shared_ptr<HoloNetPacket> handshakePacket;
	this->recvPacket(handshakePacket);

	auto hs = GetHandshakeFromPacket(handshakePacket);

	LOG4CXX_INFO(logger_, "Server name " << hs.clientName);
	LOG4CXX_INFO(logger_, "Magic number: " << hs.magicNumber);
	LOG4CXX_INFO(logger_, "Protocol version: " << hs.protocolVersion);
	LOG4CXX_INFO(logger_, "RGBAZ Mode: " << hs.rgbazWidth << "x" << hs.rgbazHeight << "@" << hs.captureFPS);
	LOG4CXX_INFO(logger_, "RGBAZ Field-of-view: " << hs.captureHOV << " deg horiz, " << hs.captureVOV << " deg vert");
	LOG4CXX_INFO(logger_, "Audio Mode: " << hs.audioNumChan << " chan, " << hs.audioBitDepth << " bits @ " << hs.audioFreq << " kHz");


	isConnected_ = true;

	start();

	return hs;
}



