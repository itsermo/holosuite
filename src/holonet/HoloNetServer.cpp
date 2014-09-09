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
	boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::tcp::v4(), port_);

	io_service_ = boost::shared_ptr<boost::asio::io_service>(new boost::asio::io_service);
	

	acceptor_ = boost::shared_ptr<boost::asio::ip::tcp::acceptor>(new boost::asio::ip::tcp::acceptor(*io_service_));

	acceptor_->open(endpoint.protocol());
	acceptor_->bind(endpoint);
	acceptor_->listen();

	while (shouldListen_)
	{
		auto socket = boost::shared_ptr<boost::asio::ip::tcp::socket>(new boost::asio::ip::tcp::socket(*io_service_));

		acceptor_->accept(*socket);

		LOG4CXX_DEBUG(logger_, "Accepted socket connection from a client" << port_);

		isConnected_ = true;

		socket->set_option(boost::asio::ip::tcp::no_delay(true));
		//socket_->set_option(boost::asio::socket_base::send_buffer_size(65536));
		//socket_->set_option(boost::asio::socket_base::receive_buffer_size(65536));

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
