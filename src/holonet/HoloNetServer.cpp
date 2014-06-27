#include "HoloNetServer.hpp"

using namespace holo;
using namespace holo::net;

HoloNetServer::HoloNetServer()
{

}

HoloNetServer::~HoloNetServer()
{

}

HoloNetProtocolHandshake HoloNetServer::listenAndWait(unsigned short port, HoloNetProtocolHandshake localInfo)
{
	LOG4CXX_INFO(logger_, "Server " << localInfo.clientName << " listening for holosuite client connections on port " << port);

	io_service_ = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service);
	socket_ = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(*io_service_));
	acceptor_ = boost::shared_ptr<boost::asio::ip::tcp::acceptor>(new boost::asio::ip::tcp::acceptor(*io_service_));

	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port);

	acceptor_->open(endpoint.protocol());
	acceptor_->bind(endpoint);
	acceptor_->listen();

	acceptor_->accept(*socket_);

	LOG4CXX_DEBUG(logger_, "Accepted socket connection from a client" << port);

	socket_->set_option(boost::asio::ip::tcp::no_delay(true));
	socket_->set_option(boost::asio::socket_base::send_buffer_size(65536));
	socket_->set_option(boost::asio::socket_base::receive_buffer_size(65536));

	this->start();

	this->performHandshake(localInfo);
	
	LOG4CXX_DEBUG(logger_, "Trying to get handshake info from client" << port);
	boost::shared_ptr<HoloNetPacket> handshakePacket;
	recvPacket(handshakePacket);

	auto hs = GetHandshakeFromPacket(handshakePacket);

	LOG4CXX_INFO(logger_, "Client " << hs.clientName << " connected." << std::endl
		<< "Magic number: " << hs.magicNumber << std::endl
		<< "Protocol version: " << hs.protocolVersion << std::endl
		<< "RGBAZ Width: " << hs.rgbazWidth << std::endl
		<< "RGBAZ Height: " << hs.rgbazHeight << std::endl
		<< "Capture FPS: " << hs.captureFPS << std::endl
		<< "Horizontal Field-of-View: " << hs.captureHOV << std::endl
		<< "Vertical Field-of-View: " << hs.captureVOV);
	return hs;
}

void HoloNetServer::disconnect()
{
	if (isConnected())
	{
		socket_->close();
		socket_.reset();
	}
}
