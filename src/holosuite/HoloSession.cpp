#include "HoloSession.hpp"
#include "../holoutils/HoloUtils.hpp"

#include <vector>
#include <chrono>
#include <ctime>


using namespace holo;
using namespace holo::net;

HoloSession::HoloSession()
{

}


HoloSession::HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
	std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> && codec,
	std::shared_ptr<holo::net::HoloNetSession> netSession, holo::net::HoloNetProtocolHandshake remoteInfo,
	std::unique_ptr<holo::render::IHoloRender> && render
	) :
	shouldCapture_(false),
	shouldEncode_(false),
	shouldDecode_(false),
	shouldRender_(false),
	isRunning_(false),
	remoteInfo_(remoteInfo),
	worldConvertCache_()
{
	capture_ = std::move(capture);
	cloudCodec_ = std::move(codec);
	netSession_ = netSession;
	render_ = std::move(render);
	LOG4CXX_DEBUG(logger_, "HoloSession object instantiated with custom objects");
}

HoloSession::HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && codec,
	std::shared_ptr<holo::net::HoloNetSession> netSession, holo::net::HoloNetProtocolHandshake remoteInfo,
	std::unique_ptr<holo::render::IHoloRender> && render
	) :
	shouldCapture_(false),
	shouldEncode_(false),
	shouldDecode_(false),
	shouldRender_(false),
	isRunning_(false),
	remoteInfo_(remoteInfo),
	worldConvertCache_()
{
	capture_ = std::move(capture);
	rgbazCodec_ = std::move(codec);
	netSession_ = netSession;
	render_ = std::move(render);
	LOG4CXX_DEBUG(logger_, "HoloSession object instantiated with custom objects");
}

bool HoloSession::start()
{
	LOG4CXX_INFO(logger_, "Starting session threads...");

	//allocate and setup remote cloud
	remoteCloud_ = HoloCloudPtr(new HoloCloud(remoteInfo_.rgbazWidth, remoteInfo_.rgbazHeight));
	remoteCloud_->is_dense = false;
	remoteCloud_->sensor_origin_.setZero();
	remoteCloud_->sensor_orientation_.setIdentity();

	//create convert cache for remote cloud so we can reproject to real-world coordinates
	worldConvertCache_.xzFactor = tan((remoteInfo_.captureHOV * M_PI / 180.0f) / 2) * 2;
	worldConvertCache_.yzFactor = tan((remoteInfo_.captureVOV * M_PI / 180.0f) / 2) * 2;
	worldConvertCache_.resolutionX = remoteInfo_.rgbazWidth;
	worldConvertCache_.resolutionY = remoteInfo_.rgbazHeight;
	worldConvertCache_.halfResX = worldConvertCache_.resolutionX / 2;
	worldConvertCache_.halfResY = worldConvertCache_.resolutionY / 2;
	worldConvertCache_.coeffX = worldConvertCache_.resolutionX / worldConvertCache_.xzFactor;
	worldConvertCache_.coeffY = worldConvertCache_.resolutionY / worldConvertCache_.yzFactor;

	shouldCapture_ = capture_ ? true : false;
	shouldRender_ = render_ ? true : false;

	shouldEncode_ = shouldCapture_.load();
	shouldDecode_ = shouldRender_.load();

	if (shouldCapture_)
		captureThread_ = std::thread(&HoloSession::captureLoop, this);

	if (shouldEncode_)
		encodeThread_ = std::thread(&HoloSession::encodeLoop, this);

	if (shouldDecode_)
		decodeThread_ = std::thread(&HoloSession::decodeLoop, this);

	if (shouldRender_)
		renderThread_ = std::thread(&HoloSession::renderLoop, this);

	netRecvThread_ = std::thread(&HoloSession::netRecvLoop, this);

	isRunning_ = true;

	return true;
}

void HoloSession::stop()
{
	isRunning_ = false;
}

bool HoloSession::isRunning()
{
	return isRunning_.load();
}

bool HoloSession::isConnected()
{
	return true;
}

HoloSession::~HoloSession()
{

}

void HoloSession::netRecvLoop()
{
	for (;;)
	{

		boost::shared_ptr<HoloNetPacket> recvPacket;
		netSession_->recvPacket(recvPacket);

		switch (recvPacket->type)
		{
		case HOLO_NET_PACKET_TYPE_HANDSHAKE:
			remoteInfo_ = holo::net::HoloNetSession::GetHandshakeFromPacket(recvPacket);
			break;
		case HOLO_NET_PACKET_TYPE_GRACEFUL_DISCONNECT:
			shouldDecode_ = false;
			break;
		case HOLO_NET_PACKET_TYPE_OCTREE_COMPRESSED:
		{
			std::unique_lock<std::mutex> rccLock(remoteCloudCompressedMutex_);
			remoteCloudCompressed_ = boost::shared_ptr<std::vector<uchar>>(new std::vector<uchar>(recvPacket->value));
			rccLock.unlock();
			haveRemoteCloudCompressed_.notify_all();
			break;
		}
		case HOLO_NET_PACKET_TYPE_AUDIO_COMPRESSED:
		{
			std::unique_lock<std::mutex> racLock(remoteAudioCompressedMutex_);
			*remoteAudioCompressed_ = recvPacket->value;
			racLock.unlock();
			haveRemoteAudioCompressed_.notify_all();
			break;
		}
		case HOLO_NET_PACKET_TYPE_RGBZ_FRAME_COMPRESSED:
		{
			std::unique_lock<std::mutex> rgbazLock(remoteRGBAZCompressedMutex_);
			remoteRGBAZCompressed_ = boost::shared_ptr<std::vector<uchar>>(new std::vector<uchar>(recvPacket->value));
			rgbazLock.unlock();
			haveRemoteRGBAZCompressed_.notify_all();
		}
		case HOLO_NET_PACKET_TYPE_UNKNOWN:
		default:
			break;
		}
	}
}

void HoloSession::captureLoop()
{
	if (rgbazCodec_)
	{
		localRGBAZ_ = boost::shared_ptr<HoloRGBAZMat>(new HoloRGBAZMat);
		localRGBAZ_->rgba = cv::Mat(480, 640, CV_8UC4);
		localRGBAZ_->z = cv::Mat(480, 640, CV_16UC1);
	}

	while (shouldCapture_)
	{

		if (rgbazCodec_)
		{
			std::unique_lock<std::mutex> ulLocalPixMaps(localRGBAZMutex_);
			capture_->waitAndGetNextFrame(localRGBAZ_->rgba, localRGBAZ_->z);
			ulLocalPixMaps.unlock();
			haveLocalRGBAZ_.notify_all();
		}
		else
		{
			HoloCloudPtr localCloud;
			capture_->waitAndGetNextPointCloud(localCloud);
			std::unique_lock<std::mutex> ulLocalCloud(localCloudMutex_);
			localCloud_ = localCloud;
			ulLocalCloud.unlock();
			haveLocalCloud_.notify_all();
		}

	}
}

void HoloSession::decodeLoop()
{
	while (shouldDecode_)
	{
		if (rgbazCodec_)
		{
			std::unique_lock<std::mutex> ulRemoteRGBAZCompressed(remoteRGBAZCompressedMutex_);
			haveRemoteRGBAZCompressed_.wait(ulRemoteRGBAZCompressed);

			boost::shared_ptr<HoloRGBAZMat> decodedMats;
			rgbazCodec_->decode(remoteRGBAZCompressed_, decodedMats);

			if (decodedMats)
			{
				std::unique_lock<std::mutex> ulRemoteRGBAZ(remoteRGBAZMutex_);
				remoteRGBAZ_ = decodedMats;
				ulRemoteRGBAZ.unlock();

				haveRemoteRGBAZ_.notify_all();
			}
		}
		else if (cloudCodec_)
		{
			std::unique_lock<std::mutex> ulRemoteCloudCompressed(remoteCloudCompressedMutex_);
			haveRemoteCloudCompressed_.wait(ulRemoteCloudCompressed);

			auto remoteCloud = boost::shared_ptr<HoloCloud>(new HoloCloud);
			cloudCodec_->decode(remoteCloudCompressed_, remoteCloud);

			std::unique_lock<std::mutex> ulRemoteCloud(remoteCloudMutex_);
			remoteCloud_ = remoteCloud;

			ulRemoteCloud.unlock();
			ulRemoteCloudCompressed.unlock();

			haveRemoteCloud_.notify_all();
		}
	}
}

void HoloSession::encodeLoop()
{

	while(shouldEncode_)
	{

		if (rgbazCodec_)
		{
			std::unique_lock<std::mutex> ulLocalRGBAZ(localRGBAZMutex_);
			haveLocalRGBAZ_.wait(ulLocalRGBAZ);

			boost::shared_ptr<std::vector<uchar>> encodedData;
			rgbazCodec_->encode(localRGBAZ_, encodedData);
			ulLocalRGBAZ.unlock();

			if (encodedData)
			{
				auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket());
				packet->type = HOLO_NET_PACKET_TYPE_RGBZ_FRAME_COMPRESSED;
				packet->length = encodedData->size();
				packet->value = *encodedData;

				netSession_->sendPacket(std::move(packet));
			}
		}
		else if (cloudCodec_)
		{
			std::unique_lock<std::mutex> ulLocalCloud(localCloudMutex_);
			haveLocalCloud_.wait(ulLocalCloud);


			boost::shared_ptr<std::vector<uchar>> encodedData;
			cloudCodec_->encode(localCloud_, encodedData);

			ulLocalCloud.unlock();

			auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket());
			packet->type = HOLO_NET_PACKET_TYPE_OCTREE_COMPRESSED;
			packet->length = encodedData->size();
			packet->value = *encodedData;

			netSession_->sendPacket(std::move(packet));
		}
	}
}

void HoloSession::renderLoop()
{
	while (shouldRender_)
	{
		if (rgbazCodec_)
		{
			std::unique_lock<std::mutex> ulRemoteRGBAZData(remoteRGBAZMutex_);
			haveRemoteRGBAZ_.wait(ulRemoteRGBAZData);
			std::unique_lock<std::mutex> ulRemoteCloud(remoteCloudMutex_);
			holo::utils::ReprojectToRealWorld(remoteCloud_, *remoteRGBAZ_, worldConvertCache_);

			ulRemoteRGBAZData.unlock();
			ulRemoteCloud.unlock();

			HoloCloudPtr renderCloud = HoloCloudPtr(new HoloCloud((const HoloCloud)*remoteCloud_));
			render_->updateFromPointCloud(std::move(renderCloud));
		}
		else
		{
			std::unique_lock<std::mutex> ulRemoteCloud(remoteCloudMutex_);
			haveRemoteCloud_.wait(ulRemoteCloud);
			HoloCloudPtr renderCloud = HoloCloudPtr(new HoloCloud((const HoloCloud)*remoteCloud_));
			ulRemoteCloud.unlock();

			render_->updateFromPointCloud(std::move(renderCloud));
		}

	}
}