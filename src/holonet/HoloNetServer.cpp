#include "HoloNetServer.hpp"

using namespace holo;
using namespace holo::net;

HoloNetServer::HoloNetServer() : HoloNetSession()
{
	 logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.net.server");
}

HoloNetServer::~HoloNetServer()
{
	disconnect();
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

	isConnected_ = true;

	socket_->set_option(boost::asio::ip::tcp::no_delay(true));
	//socket_->set_option(boost::asio::socket_base::send_buffer_size(65536));
	//socket_->set_option(boost::asio::socket_base::receive_buffer_size(65536));

	this->performHandshake(localInfo);
	
	LOG4CXX_DEBUG(logger_, "Trying to get handshake info from client" << port);
	boost::shared_ptr<HoloNetPacket> handshakePacket;
	recvPacket(handshakePacket);

	auto hs = GetHandshakeFromPacket(handshakePacket);

	LOG4CXX_INFO(logger_, "Client " << hs.clientName << " connected.");
	LOG4CXX_INFO(logger_, "Magic number: " << hs.magicNumber);
	LOG4CXX_INFO(logger_, "Protocol version: " << hs.protocolVersion);
	LOG4CXX_INFO(logger_, "RGBAZ Mode: " << hs.rgbazWidth << "x" << hs.rgbazHeight << "@" << hs.captureFPS);
	LOG4CXX_INFO(logger_, "RGBAZ Field-of-view: " << hs.captureHOV << " deg horiz, " << hs.captureVOV << " deg vert");
	LOG4CXX_INFO(logger_, "Audio Mode: " << hs.audioNumChan << " chan, " << hs.audioBitDepth << " bits @ " << hs.audioFreq <<" kHz");


	start();

	return hs;
}

//void HoloNetServer::disconnect()
//{
//
//	if (socket_ && isConnected())
//	{
//		socket_->shutdown(boost::asio::socket_base::shutdown_both);
//		socket_->close();
//		socket_.reset();
//	}
//
//	isConnected_ = false;
//}
