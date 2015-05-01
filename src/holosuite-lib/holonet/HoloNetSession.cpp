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

#ifdef ENABLE_HOLO_UDT
void HoloNetSession::performHandshake(HoloNetProtocolHandshake localInfo, UDTSOCKET socket)
{
	LOG4CXX_DEBUG(logger_, "Sending handshake info");

	auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);
	packet->type = HOLO_NET_PACKET_TYPE_HANDSHAKE;
	packet->length = sizeof(HoloNetProtocolHandshake);
	packet->value = std::vector<uint8_t>(sizeof(HoloNetProtocolHandshake));

	localInfo.magicNumber = boost::asio::detail::socket_ops::host_to_network_long(HOLO_NET_MAGIC_NUMBER);
	localInfo.protocolVersion = boost::asio::detail::socket_ops::host_to_network_long(HOLO_NET_CURRENT_PROTOCOL_VERSION);
	localInfo.rgbazWidth = boost::asio::detail::socket_ops::host_to_network_long(localInfo.rgbazWidth);
	localInfo.rgbazHeight = boost::asio::detail::socket_ops::host_to_network_long(localInfo.rgbazHeight);
	localInfo.captureFPS = boost::asio::detail::socket_ops::host_to_network_long(static_cast<u_long>(localInfo.captureFPS));
	localInfo.captureHOV = boost::asio::detail::socket_ops::host_to_network_long(static_cast<u_long>(localInfo.captureHOV));
	localInfo.captureVOV = boost::asio::detail::socket_ops::host_to_network_long(static_cast<u_long>(localInfo.captureVOV));
	localInfo.videoCodecType = boost::asio::detail::socket_ops::host_to_network_long(localInfo.videoCodecType);
	localInfo.audioCodecType = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioCodecType);
	localInfo.audioBitDepth = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioBitDepth);
	localInfo.audioNumChan = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioNumChan);
	localInfo.audioFreq = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioFreq);

	memcpy(packet->value.data(), &localInfo, sizeof(localInfo));

	sendPacket(packet, socket);
}

void HoloNetSession::sendPacket(boost::shared_ptr<HoloNetPacket> & packet, UDTSOCKET socket)
{
	int ret = 0;

	int dataLength = packet->length;

	packet->type = boost::asio::detail::socket_ops::host_to_network_long(packet->type);
	packet->length = boost::asio::detail::socket_ops::host_to_network_long(packet->length);

	//boost::asio::write(*socket, boost::asio::buffer(&packet->type, sizeof(uint32_t)* 2), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
	ret = UDT::send(socket, (const char*)&packet->type, sizeof(uint32_t)* 2, 0);
	if (ret == UDT::ERROR)
		throw boost::system::system_error(boost::asio::error::interrupted);

	int remaining = dataLength;
	while (remaining > 0)
	{
		ret = UDT::send(socket, (const char*)packet->value.data() + (dataLength - remaining), remaining, 0);
		if (ret == UDT::ERROR)
			throw boost::system::system_error(boost::asio::error::interrupted);

		remaining -= ret;
	}
	packet->type = boost::asio::detail::socket_ops::network_to_host_long(packet->type);
	packet->length = boost::asio::detail::socket_ops::network_to_host_long(packet->length);
}

void HoloNetSession::recvPacket(boost::shared_ptr<HoloNetPacket> & packet, UDTSOCKET socket)
{
	packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);

	int ret = 0;
	//boost::system::error_code error;
	size_t received = 0;

	std::vector<uint32_t> typeLength(sizeof(uint32_t)* 2);
	//boost::asio::read(*socket, boost::asio::buffer(typeLength), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
	ret = UDT::recv(socket, (char*)typeLength.data(), sizeof(uint32_t)* 2, 0);
	if (ret == UDT::ERROR)
		throw boost::system::system_error(boost::asio::error::interrupted);


	packet->type = boost::asio::detail::socket_ops::network_to_host_long(typeLength[0]);
	packet->length = boost::asio::detail::socket_ops::network_to_host_long(typeLength[1]);

	packet->value = std::vector<uint8_t>(packet->length);

	int remaining = packet->length;
	while (remaining > 0)
	{
		//boost::asio::read(*socket, boost::asio::buffer(packet->value), boost::asio::transfer_exactly(packet->length), error);
		ret = UDT::recv(socket, (char*)packet->value.data() + (packet->length - remaining), remaining, 0);
		if (ret == UDT::ERROR)
			throw boost::system::system_error(boost::asio::error::interrupted);

		remaining -= ret;
	}
}

#else

void HoloNetSession::sendPacket(boost::shared_ptr<HoloNetPacket> & packet, boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
		boost::system::error_code error;

		int dataLength = packet->length;
	
		packet->type = boost::asio::detail::socket_ops::host_to_network_long(packet->type);
		packet->length = boost::asio::detail::socket_ops::host_to_network_long(packet->length);

		boost::asio::write(*socket, boost::asio::buffer(&packet->type, sizeof(uint32_t)* 2), boost::asio::transfer_exactly(sizeof(uint32_t)* 2), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);

		boost::asio::write(*socket, boost::asio::buffer(packet->value, dataLength), boost::asio::transfer_exactly(dataLength), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);

		packet->type = boost::asio::detail::socket_ops::network_to_host_long(packet->type);
		packet->length = boost::asio::detail::socket_ops::network_to_host_long(packet->length);

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


		packet->type = boost::asio::detail::socket_ops::network_to_host_long(typeLength[0]);
		packet->length = boost::asio::detail::socket_ops::network_to_host_long(typeLength[1]);

		packet->value = std::vector<uint8_t>(packet->length);

		boost::asio::read(*socket, boost::asio::buffer(packet->value), boost::asio::transfer_exactly(packet->length), error);
		if (error)
			throw boost::system::system_error(boost::asio::error::interrupted);
}

void HoloNetSession::performHandshake(HoloNetProtocolHandshake localInfo, boost::shared_ptr<boost::asio::ip::tcp::socket> socket)
{
	LOG4CXX_DEBUG(logger_, "Sending handshake info");

	auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);
	packet->type = HOLO_NET_PACKET_TYPE_HANDSHAKE;
	packet->length = sizeof(HoloNetProtocolHandshake);
	packet->value = std::vector<uint8_t>(sizeof(HoloNetProtocolHandshake));

	localInfo.magicNumber = boost::asio::detail::socket_ops::host_to_network_long(HOLO_NET_MAGIC_NUMBER);
	localInfo.protocolVersion = boost::asio::detail::socket_ops::host_to_network_long(HOLO_NET_CURRENT_PROTOCOL_VERSION);
	localInfo.rgbazWidth = boost::asio::detail::socket_ops::host_to_network_long(localInfo.rgbazWidth);
	localInfo.rgbazHeight = boost::asio::detail::socket_ops::host_to_network_long(localInfo.rgbazHeight);
	localInfo.captureFPS = boost::asio::detail::socket_ops::host_to_network_long(static_cast<u_long>(localInfo.captureFPS));
	localInfo.captureHOV = boost::asio::detail::socket_ops::host_to_network_long(static_cast<u_long>(localInfo.captureHOV));
	localInfo.captureVOV = boost::asio::detail::socket_ops::host_to_network_long(static_cast<u_long>(localInfo.captureVOV));
	localInfo.videoCodecType = boost::asio::detail::socket_ops::host_to_network_long(localInfo.videoCodecType);
	localInfo.audioCodecType = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioCodecType);
	localInfo.audioBitDepth = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioBitDepth);
	localInfo.audioNumChan = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioNumChan);
	localInfo.audioFreq = boost::asio::detail::socket_ops::host_to_network_long(localInfo.audioFreq);

	memcpy(packet->value.data(), &localInfo, sizeof(localInfo));

	sendPacket(packet, socket);
}
#endif

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
#ifdef ENABLE_HOLO_UDT
			auto disconnectPacket = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);
			disconnectPacket->type = HOLO_NET_PACKET_TYPE_GRACEFUL_DISCONNECT;
			disconnectPacket->length = 0;

			try{
				this->sendPacket(std::move(disconnectPacket));
			}
			catch (boost::system::system_error) {
				LOG4CXX_WARN(logger_, "Could not send graceful disconnect packet")
			}

			UDT::close(sockets_[i]);
#else
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
#endif
		}

		sockets_.clear();
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

	}
}

HoloNetProtocolHandshake HoloNetSession::GetHandshakeFromPacket(boost::shared_ptr<HoloNetPacket> & packet)
{
	HoloNetProtocolHandshake hs;

	memcpy(&hs, packet->value.data(), sizeof(hs));

	hs.magicNumber = boost::asio::detail::socket_ops::network_to_host_long(hs.magicNumber);
	hs.protocolVersion = boost::asio::detail::socket_ops::network_to_host_long(hs.protocolVersion);
	hs.rgbazWidth = boost::asio::detail::socket_ops::network_to_host_long(hs.rgbazWidth);
	hs.rgbazHeight = boost::asio::detail::socket_ops::network_to_host_long(hs.rgbazHeight);
	hs.captureFPS = static_cast<float>(boost::asio::detail::socket_ops::network_to_host_long(hs.captureFPS));
	hs.captureHOV = static_cast<float>(boost::asio::detail::socket_ops::network_to_host_long(hs.captureHOV));
	hs.captureVOV = static_cast<float>(boost::asio::detail::socket_ops::network_to_host_long(hs.captureVOV));
	hs.videoCodecType = boost::asio::detail::socket_ops::network_to_host_long(hs.videoCodecType);
	hs.audioCodecType = boost::asio::detail::socket_ops::network_to_host_long(hs.audioCodecType);
	hs.audioBitDepth = boost::asio::detail::socket_ops::network_to_host_long(hs.audioBitDepth);
	hs.audioNumChan = boost::asio::detail::socket_ops::network_to_host_long(hs.audioNumChan);
	hs.audioFreq = boost::asio::detail::socket_ops::network_to_host_long(hs.audioFreq);
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