#include "HoloNetSession.hpp"
#include <future>

using namespace holo;
using namespace holo::net;

HoloNetSession::HoloNetSession() : isConnected_(false), shouldSend_(false)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.net.session");

	LOG4CXX_DEBUG(logger_, "HoloNetSession object instantiated")
}

HoloNetSession::~HoloNetSession()
{
	LOG4CXX_DEBUG(logger_, "HoloNetSession object destroyed")
}


bool HoloNetSession::isConnected()
{
	return isConnected_;
}

void HoloNetSession::sendPacketAsync(boost::shared_ptr<HoloNetPacket> && packet)
{
	if (isConnected())
	{
		std::unique_lock<std::mutex> ulSendQueue(sendQueueMutex_);
		pushLocalPacket(std::move(packet));
		ulSendQueue.unlock();
		haveLocalPacketCV_.notify_all();
	}
	else
		throw boost::system::system_error(boost::asio::error::not_connected);
}

void HoloNetSession::sendPacket(boost::shared_ptr<HoloNetPacket> && packet)
{
	//std::lock_guard<std::mutex> lg(socketListMutex_);
	for (int i = 0; i < sockets_.size(); i++)
	{
		sendPacket(packet, sockets_[i]);
	}
}

void HoloNetSession::recvPacket(boost::shared_ptr<HoloNetPacket> & packet)
{
	//std::lock_guard<std::mutex> lg(socketListMutex_);
	recvPacket(packet, sockets_[0]);
}

void HoloNetSession::sendPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
		boost::system::error_code error;

		int dataLength = packet->length;

		packet->type = htonl(packet->type);
		packet->length = htonl(packet->length);

		boost::asio::write(*socket, boost::asio::buffer(&packet->type, sizeof(uint32_t)* 2), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);

		boost::asio::write(*socket, boost::asio::buffer(packet->value, dataLength), boost::asio::transfer_exactly(dataLength), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);

		packet->type = ntohl(packet->type);
		packet->length = ntohl(packet->length);

}

void HoloNetSession::recvPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
		packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);

		boost::system::error_code error;
		size_t received = 0;

		std::vector<uint32_t> typeLength(sizeof(uint32_t)* 2);
		boost::asio::read(*socket, boost::asio::buffer(typeLength), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);


		packet->type = ntohl(typeLength[0]);
		packet->length = ntohl(typeLength[1]);

		packet->value = std::vector<uint8_t>(packet->length);

		boost::asio::read(*socket, boost::asio::buffer(packet->value), boost::asio::transfer_exactly(packet->length), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);
}

void HoloNetSession::disconnect()
{
	if (shouldSend_)
	{
		shouldSend_ = false;
		sendQueueThread_.join();
	}

	if (isConnected())
	{
		for (int i = 0; i < sockets_.size(); i++)
		{
			if (sockets_[i])
			{
				if (sockets_[i]->is_open())
				{
					auto disconnectPacket = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);
					disconnectPacket->type = HOLO_NET_PACKET_TYPE_GRACEFUL_DISCONNECT;
					disconnectPacket->length = 0;

					try{
						this->sendPacket(std::move(disconnectPacket));
					}
					catch (boost::system::system_error) {
						LOG4CXX_WARN(logger_, "Could not send graceful disconnect packet")
					}

					boost::system::error_code error;
					sockets_[i]->shutdown(boost::asio::socket_base::shutdown_both, error);
					sockets_[i]->close();
				}

				sockets_[i].reset();
			}
		}
	}

	isConnected_ = false;
}

void HoloNetSession::pushLocalPacket(boost::shared_ptr<HoloNetPacket> && packet)
{
	if (sendQueue_.size() < HOLO_NET_PACKET_BUFFER_SIZE)
	{
		//std::lock_guard<std::mutex> queuelockguard(sendQueueMutex_);
		sendQueue_.push(packet);
		//haveLocalPacketCV_.notify_all();
	}
	else
		LOG4CXX_WARN(logger_, "local packets are dropping before sending");
}

void HoloNetSession::popLocalPacket(boost::shared_ptr<HoloNetPacket> & packet)
{
	if (!sendQueue_.empty())
	{
//		std::lock_guard<std::mutex> queueLockGuard(sendQueueMutex_);
		packet = sendQueue_.front();
		sendQueue_.pop();
	}
	else
		packet = nullptr;
}

//void HoloNetSession::pushRemotePacket(boost::shared_ptr<HoloNetPacket> && packet)
//{
//	if (recvQueue_.size() < HOLO_NET_PACKET_BUFFER_SIZE)
//	{
//		std::lock_guard<std::mutex> queueLockGuard(recvQueueMutex_);
//		recvQueue_.push(packet);
//	}
//	else
//		logger_->warn("Remote packets are being dropped before local processing");
//
//	haveNewRemotePacket_.notify_all();
//}

//void HoloNetSession::popRemotePacket(boost::shared_ptr<HoloNetPacket> & packet)
//{
//	if (!recvQueue_.empty())
//	{
//		std::lock_guard<std::mutex> queueUL(recvQueueMutex_);
//		packet = recvQueue_.front();
//		recvQueue_.pop();
//	}
//	else
//	{
//		std::unique_lock<std::mutex> queueUL(recvQueueMutex_);
//		haveNewRemotePacket_.wait(queueUL);
//		packet = recvQueue_.front();
//		recvQueue_.pop();
//		queueUL.unlock();
//	}
//
//}
//

void HoloNetSession::sendLoop()
{
	boost::system::error_code error;
	boost::shared_ptr<HoloNetPacket> packet;

	while (shouldSend_)
	{
		auto sendPacketLock = std::unique_lock<std::mutex>(sendQueueMutex_);
		if (std::cv_status::timeout == haveLocalPacketCV_.wait_for(sendPacketLock, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
			continue;

		for (size_t i = 0; i < sendQueue_.size(); i++)
		{
			popLocalPacket(packet);

			try{
				this->sendPacket(std::move(packet));
			}
			catch (boost::system::system_error) {
				return;
			}
		}

		sendPacketLock.unlock();

		//if (packet)
		//{
		//	int dataLength = packet->length;

		//	packet->type = htonl(packet->type);
		//	packet->length = htonl(packet->length);

		//	boost::asio::write(*socket_, boost::asio::buffer(&packet->type, sizeof(uint32_t)* 2), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
		//	if (error)
		//		throw boost::system::system_error(boost::asio::error::interrupted);

		//	boost::asio::write(*socket_, boost::asio::buffer(packet->value, dataLength), boost::asio::transfer_exactly(dataLength), error);

		//	if (error)
		//		throw boost::system::system_error(boost::asio::error::interrupted);
		//}

	}
}

//void HoloNetSession::recvLoop()
//{
//	boost::system::error_code error;
//
//	while (isConnected())
//	{
//		auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);
//		
//		size_t received = 0;
//
//		std::vector<uint32_t> typeLength(sizeof(uint32_t)* 2);
//		boost::asio::read(*socket_, boost::asio::buffer(typeLength), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
//		if (error)
//			throw boost::system::system_error(boost::asio::error::interrupted);
//
//
//		packet->type = ntohl(typeLength[0]);
//		packet->length = ntohl(typeLength[1]);
//
//		packet->value = std::vector<uint8_t>(packet->length);
//
//		boost::asio::read(*socket_, boost::asio::buffer(packet->value), boost::asio::transfer_exactly(packet->length), error);
//		if (error)
//			throw boost::system::system_error(boost::asio::error::interrupted);
//
//		pushRemotePacket(std::move(packet));
//
//	}
//}

void HoloNetSession::performHandshake(HoloNetProtocolHandshake localInfo, boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
	LOG4CXX_DEBUG(logger_, "Sending handshake info");

	auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);
	packet->type = HOLO_NET_PACKET_TYPE_HANDSHAKE;
	packet->length = sizeof(HoloNetProtocolHandshake);
	packet->value = std::vector<uint8_t>(sizeof(HoloNetProtocolHandshake));

	localInfo.magicNumber = htonl(HOLO_NET_MAGIC_NUMBER);
	localInfo.protocolVersion = htonl(HOLO_NET_CURRENT_PROTOCOL_VERSION);
	localInfo.rgbazWidth = htonl(localInfo.rgbazWidth);
	localInfo.rgbazHeight = htonl(localInfo.rgbazHeight);
	localInfo.captureFPS = htonl(static_cast<u_long>(localInfo.captureFPS));
	localInfo.captureHOV = htonl(static_cast<u_long>(localInfo.captureHOV));
	localInfo.captureVOV = htonl(static_cast<u_long>(localInfo.captureVOV));
	localInfo.videoCodecType = htonl(localInfo.videoCodecType);
	localInfo.audioCodecType = ntohl(localInfo.audioCodecType);
	localInfo.audioBitDepth = ntohl(localInfo.audioBitDepth);
	localInfo.audioNumChan = ntohl(localInfo.audioNumChan);
	localInfo.audioFreq = ntohl(localInfo.audioFreq);

	memcpy(packet->value.data(), &localInfo, sizeof(localInfo));

	sendPacket(packet, socket);
}

HoloNetProtocolHandshake HoloNetSession::GetHandshakeFromPacket(boost::shared_ptr<HoloNetPacket> & packet)
{
	HoloNetProtocolHandshake hs;

	memcpy(&hs, packet->value.data(), sizeof(hs));

	hs.magicNumber = ntohl(hs.magicNumber);
	hs.protocolVersion = ntohl(hs.protocolVersion);
	hs.rgbazWidth = ntohl(hs.rgbazWidth);
	hs.rgbazHeight = ntohl(hs.rgbazHeight);
	hs.captureFPS = static_cast<float>(ntohl(hs.captureFPS));
	hs.captureHOV = static_cast<float>(ntohl(hs.captureHOV));
	hs.captureVOV = static_cast<float>(ntohl(hs.captureVOV));
	hs.videoCodecType = ntohl(hs.videoCodecType);
	hs.audioCodecType = ntohl(hs.audioCodecType);
	hs.audioBitDepth = ntohl(hs.audioBitDepth);
	hs.audioNumChan = ntohl(hs.audioNumChan);
	hs.audioFreq = ntohl(hs.audioFreq);
	return hs;
}

void HoloNetSession::start()
{
	if (!shouldSend_)
	{
		shouldSend_ = true;
		this->sendQueueThread_ = std::thread(&HoloNetSession::sendLoop, this);
		//this->recvQueueThread_ = std::thread(&HoloNetSession::recvLoop, this);
	}

}