#include "HoloSession.hpp"
#include "../holoutils/HoloUtils.hpp"

#include <vector>
#include <chrono>
#include <ctime>
#include <future>

#include <boost/smart_ptr.hpp>

using namespace holo;
using namespace holo::net;

HoloSession::HoloSession()
{

}

HoloSession::HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
	std::unique_ptr<holo::capture::IHoloCaptureAudio> && audioCapture,
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && encoderRGBAZ,
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && decoderRGBAZ,
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> && encoderCloud,
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> && decoderCloud,
	std::unique_ptr<holo::codec::IHoloCodec<std::vector<uchar>>> && audioEncoder,
	std::unique_ptr<holo::codec::IHoloCodec<std::vector<uchar>>> && audioDecoder,
	std::unique_ptr<holo::render::IHoloRender> && render,
	std::unique_ptr<holo::render::IHoloRenderAudio> && audioRender,
	std::shared_ptr<holo::net::HoloNetSession> netSession, holo::net::HoloNetProtocolHandshake remoteInfo
	) :
	shouldCapture_(false),
	shouldEncode_(false),
	shouldDecode_(false),
	shouldRender_(false),
	isRunning_(false),
	remoteInfo_(remoteInfo),
	worldConvertCache_()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.session");

	capture_ = std::move(capture);
	audioCapture_ = std::move(audioCapture);
	rgbazDecoder_ = std::move(decoderRGBAZ);
	rgbazEncoder_ = std::move(encoderRGBAZ);
	cloudDecoder_ = std::move(decoderCloud);
	cloudEncoder_ = std::move(encoderCloud);
	audioEncoder_ = std::move(audioEncoder);
	audioDecoder_ = std::move(audioDecoder);
	netSession_ = netSession;
	render_ = std::move(render);
	audioRender_ = std::move(audioRender);

	LOG4CXX_DEBUG(logger_, "HoloSession object instantiated with custom objects");
}

bool HoloSession::start()
{
	LOG4CXX_INFO(logger_, "Starting session threads...");

	haveLocalCloud_ = false;
	haveLocalRGBAZ_ = false;
	haveRemoteCloud_ = false;
	haveRemoteCloudCompressed_ = false;
	haveRemoteAudio_ = false;
	haveRemoteAudioCompressed_ = false;
	haveRemoteRGBAZ_ = false;
	haveRemoteRGBAZCompressed_ = false;

	//allocate and setup remote cloud
	remoteCloud_ = HoloCloudPtr(new HoloCloud(remoteInfo_.rgbazWidth, remoteInfo_.rgbazHeight));
	remoteCloud_->is_dense = false;
	remoteCloud_->sensor_origin_.setZero();
	remoteCloud_->sensor_orientation_.setIdentity();

	//remoteAudioCompressed_ = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>());

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

	shouldRecv_ = netSession_ ? true : false;

	shouldCaptureAudio_ = audioCapture_ ? true : false;
	shouldEncodeAudio_ = shouldCaptureAudio_.load();

	shouldRenderAudio_ = audioRender_ ? true : false;
	shouldDecodeAudio_ = shouldRenderAudio_.load();

	if (shouldCapture_)
		captureThread_ = std::thread(&HoloSession::captureLoop, this);

	if (shouldEncode_)
		encodeThread_ = std::thread(&HoloSession::encodeLoop, this);

	if (shouldDecode_)
		decodeThread_ = std::thread(&HoloSession::decodeLoop, this);

	if (shouldRender_)
		renderThread_ = std::thread(&HoloSession::renderLoop, this);

	if (shouldRecv_)
		netRecvThread_ = std::thread(&HoloSession::netRecvLoop, this);

	if (shouldCaptureAudio_)
		captureAudioThread_ = std::thread(&HoloSession::captureAudioLoop, this);

	if (shouldEncodeAudio_)
		encodeAudioThread_ = std::thread(&HoloSession::encodeAudioLoop, this);

	if (shouldRenderAudio_)
		renderAudioThread_ = std::thread(&HoloSession::renderAudioLoop, this);

	if (shouldDecodeAudio_)
		decodeAudioThread_ = std::thread(&HoloSession::decodeAudioLoop, this);

	isRunning_ = true;

	return isRunning_;
}

void HoloSession::stop()
{
	if (shouldCapture_)
	{
		shouldCapture_ = false;
		captureThread_.join();
	}

	if (shouldRecv_)
	{
		shouldRecv_ = false;
		netRecvThread_.join();
	}

	if (shouldRender_)
	{
		shouldRender_ = false;
		renderThread_.join();
	}

	if (shouldDecode_)
	{
		shouldDecode_ = false;
		decodeThread_.join();
	}

	if (shouldEncode_)
	{
		shouldEncode_ = false;
		encodeThread_.join();
	}

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
	stop();
}

void HoloSession::netRecvLoop()
{
	boost::shared_ptr<HoloNetPacket> recvPacket;

	while (shouldRecv_)
	{

		try{
			netSession_->recvPacket(recvPacket);
		}
		catch (boost::system::system_error error)
		{
			std::async(std::launch::async, &HoloSession::stop, this);
			break;
		}

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
			if (remoteCloudCompressed_.size() < HOLO_NET_PACKET_BUFFER_SIZE)
				remoteCloudCompressed_.push(boost::make_shared <std::vector<uchar>>(recvPacket->value));
			else
				LOG4CXX_WARN(logger_, "Dropping compressed point cloud packets from receive function")
			//remoteCloudCompressed_ = boost::shared_ptr<std::vector<uchar>>(new std::vector<uchar>(recvPacket->value));
			rccLock.unlock();
			haveRemoteCloudCompressed_ = true;
			haveRemoteCloudCompressedCV_.notify_all();
			break;
		}
		case HOLO_NET_PACKET_TYPE_AUDIO_COMPRESSED:
		{
			std::unique_lock<std::mutex> racLock(remoteAudioCompressedMutex_);
			//*remoteAudioCompressed_ = recvPacket->value;
			if (remoteAudioCompressed_.size() < HOLO_NET_PACKET_BUFFER_SIZE)
				remoteAudioCompressed_.push(boost::make_shared<std::vector<uchar>>(recvPacket->value));
			else
				LOG4CXX_WARN(logger_, "Dropping compressed audio packets from receive function")
			racLock.unlock();
			haveRemoteAudioCompressed_ = true;
			haveRemoteAudioCompressedCV_.notify_all();
			break;
		}
		case HOLO_NET_PACKET_TYPE_RGBZ_FRAME_COMPRESSED:
		{
			std::unique_lock<std::mutex> rgbazLock(remoteRGBAZCompressedMutex_);
			if (remoteRGBAZCompressed_.size() < HOLO_NET_PACKET_BUFFER_SIZE)
				remoteRGBAZCompressed_.push(boost::make_shared<std::vector<uchar>>(recvPacket->value));
			else
				LOG4CXX_WARN(logger_, "Dropping compressed RGBAZ packets from receive function")
			rgbazLock.unlock();
			haveRemoteRGBAZCompressed_ = true;
			haveRemoteRGBAZCompressedCV_.notify_all();
		}
		case HOLO_NET_PACKET_TYPE_UNKNOWN:
		default:
			break;
		}
	}
}

void HoloSession::captureLoop()
{
	if (rgbazEncoder_)
	{
		localRGBAZ_ = boost::shared_ptr<HoloRGBAZMat>(new HoloRGBAZMat);
		localRGBAZ_->rgba = cv::Mat(remoteInfo_.rgbazHeight, remoteInfo_.rgbazWidth, CV_8UC4);
		localRGBAZ_->z = cv::Mat(remoteInfo_.rgbazHeight, remoteInfo_.rgbazWidth, CV_16UC1);
	}

	while (shouldCapture_)
	{

		if (rgbazEncoder_)
		{
			std::unique_lock<std::mutex> ulLocalPixMaps(localRGBAZMutex_);
			capture_->waitAndGetNextFrame(localRGBAZ_->rgba, localRGBAZ_->z);
			ulLocalPixMaps.unlock();
			haveLocalRGBAZ_ = true;
			haveLocalRGBAZCV_.notify_all();
		}
		else if (cloudEncoder_)
		{
			HoloCloudPtr localCloud;
			capture_->waitAndGetNextPointCloud(localCloud);
			std::unique_lock<std::mutex> ulLocalCloud(localCloudMutex_);
			localCloud_ = localCloud;
			ulLocalCloud.unlock();
			haveLocalCloud_ = true;
			haveLocalCloudCV_.notify_all();
		}
		else
			shouldCapture_ = false;

#ifndef WIN32
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
#endif

	}
}

void HoloSession::decodeLoop()
{
	while (shouldDecode_)
	{
		if (rgbazDecoder_)
		{
			std::unique_lock<std::mutex> ulRemoteRGBAZCompressed(remoteRGBAZCompressedMutex_);
			if (!haveRemoteRGBAZCompressed_)
			{
				if (std::cv_status::timeout == haveRemoteRGBAZCompressedCV_.wait_for(ulRemoteRGBAZCompressed, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
					continue;
			}

			boost::shared_ptr<HoloRGBAZMat> decodedMats;

			auto compressedData = remoteRGBAZCompressed_.front();
			remoteRGBAZCompressed_.pop();
			haveRemoteRGBAZCompressed_ = remoteRGBAZCompressed_.size() == 0 ? false : true;
			ulRemoteRGBAZCompressed.unlock();

			rgbazDecoder_->decode(compressedData, decodedMats);

			if (decodedMats)
			{
				std::unique_lock<std::mutex> ulRemoteRGBAZ(remoteRGBAZMutex_);
				remoteRGBAZ_ = decodedMats;
				haveRemoteRGBAZ_ = true;
				ulRemoteRGBAZ.unlock();
				haveRemoteRGBAZCV_.notify_all();
			}
		}
		else if (cloudDecoder_)
		{
			std::unique_lock<std::mutex> ulRemoteCloudCompressed(remoteCloudCompressedMutex_);
			if (!haveRemoteCloudCompressed_)
			{
				if (std::cv_status::timeout == haveRemoteCloudCompressedCV_.wait_for(ulRemoteCloudCompressed, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
					continue;
			}

			auto compressedData = remoteCloudCompressed_.front();
			remoteCloudCompressed_.pop();
			haveRemoteCloudCompressed_ = remoteCloudCompressed_.size() == 0 ? false : true;
			ulRemoteCloudCompressed.unlock();

			auto remoteCloud = boost::shared_ptr<HoloCloud>(new HoloCloud);
			cloudDecoder_->decode(compressedData, remoteCloud);

			std::unique_lock<std::mutex> ulRemoteCloud(remoteCloudMutex_);
			remoteCloud_ = remoteCloud;

			ulRemoteCloud.unlock();


			
			haveRemoteCloud_ = true;

			haveRemoteCloudCV_.notify_all();
		}
	}
}

void HoloSession::encodeLoop()
{

	while(shouldEncode_)
	{

		if (rgbazEncoder_)
		{
			std::unique_lock<std::mutex> ulLocalRGBAZ(localRGBAZMutex_);
			if (!haveLocalRGBAZ_)
			{
				if (std::cv_status::timeout == haveLocalRGBAZCV_.wait_for(ulLocalRGBAZ, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
					continue;
			}
				

			boost::shared_ptr<std::vector<uchar>> encodedData;
			rgbazEncoder_->encode(localRGBAZ_, encodedData);
			
			haveLocalRGBAZ_ = false;
			ulLocalRGBAZ.unlock();

			if (encodedData)
			{
				auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket());
				packet->type = HOLO_NET_PACKET_TYPE_RGBZ_FRAME_COMPRESSED;
				packet->length = encodedData->size();
				packet->value = *encodedData;

				try{
					netSession_->sendPacketAsync(std::move(packet));
				}
				catch (boost::system::system_error error)
				{
					std::async(std::launch::async, &HoloSession::stop, this);
					break;
				}
			}
		}
		else if (cloudEncoder_)
		{
			std::unique_lock<std::mutex> ulLocalCloud(localCloudMutex_);
			if (!haveLocalCloud_)
			{
				if (std::cv_status::timeout == haveLocalCloudCV_.wait_for(ulLocalCloud, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
					continue;
			}


			boost::shared_ptr<std::vector<uchar>> encodedData;
			cloudEncoder_->encode(localCloud_, encodedData);

			haveLocalCloud_ = false;

			ulLocalCloud.unlock();

			auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket());
			packet->type = HOLO_NET_PACKET_TYPE_OCTREE_COMPRESSED;
			packet->length = encodedData->size();
			packet->value = *encodedData;

			netSession_->sendPacketAsync(std::move(packet));
		}
	}
}

void HoloSession::renderLoop()
{
	while (shouldRender_)
	{
		if (rgbazDecoder_)
		{
			std::unique_lock<std::mutex> ulRemoteRGBAZData(remoteRGBAZMutex_);
			if (!haveRemoteRGBAZ_)
			{
				if (std::cv_status::timeout == haveRemoteRGBAZCV_.wait_for(ulRemoteRGBAZData, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
					continue;
			}


			std::unique_lock<std::mutex> ulRemoteCloud(remoteCloudMutex_);
			holo::utils::ReprojectToRealWorld(remoteCloud_, *remoteRGBAZ_, worldConvertCache_);

			haveRemoteRGBAZ_ = false;

			ulRemoteRGBAZData.unlock();
			ulRemoteCloud.unlock();

			HoloCloudPtr renderCloud = HoloCloudPtr(new HoloCloud((const HoloCloud)*remoteCloud_));
			render_->updateFromPointCloud(std::move(renderCloud));
		}
		else
		{
			std::unique_lock<std::mutex> ulRemoteCloud(remoteCloudMutex_);
			if (!haveRemoteCloud_)
			{
				if(std::cv_status::timeout == haveRemoteCloudCV_.wait_for(ulRemoteCloud, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
					continue;
			}

			HoloCloudPtr renderCloud = HoloCloudPtr(new HoloCloud((const HoloCloud)*remoteCloud_));
			
			haveRemoteCloud_ = false;
			ulRemoteCloud.unlock();

			render_->updateFromPointCloud(std::move(renderCloud));
		}

	}
}

void HoloSession::captureAudioLoop()
{
	localAudio_ = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>);

	while (shouldCaptureAudio_)
	{
		std::unique_lock<std::mutex> ulLocalAudio(localAudioMutex_);
		audioCapture_->waitAndGetNextChunk(*localAudio_);
		ulLocalAudio.unlock();
		haveLocalAudio_ = true;
		haveLocalAudioCV_.notify_all();
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void HoloSession::encodeAudioLoop()
{
	auto encData = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>);

	while (shouldEncodeAudio_)
	{
		std::unique_lock<std::mutex> ulLocalAudio(localAudioMutex_);
		if (!haveLocalAudio_)
		{
			if (std::cv_status::timeout == haveLocalAudioCV_.wait_for(ulLocalAudio, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
				continue;
		}

		auto rawData = boost::shared_ptr<std::vector<unsigned char>>(localAudio_);

		audioEncoder_->encode(rawData, encData);
		
		haveLocalAudio_ = false;
		ulLocalAudio.unlock();

		auto packet = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket());
		packet->type = HOLO_NET_PACKET_TYPE_AUDIO_COMPRESSED;
		packet->length = encData->size();
		packet->value = *encData;

		netSession_->sendPacketAsync(std::move(packet));
	}

}

void HoloSession::decodeAudioLoop()
{
	remoteAudio_ = boost::shared_ptr<std::vector<uchar>>(new std::vector<uchar>());

	while (shouldDecodeAudio_)
	{
		std::unique_lock<std::mutex> ulRemoteAudioCompressed(remoteAudioCompressedMutex_);
		if (!haveRemoteAudioCompressed_)
		{
			if (std::cv_status::timeout == haveRemoteAudioCompressedCV_.wait_for(ulRemoteAudioCompressed, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
				continue;
		}

		std::unique_lock<std::mutex> ulRemoteAudio(remoteAudioMutex_);
		auto rawData = remoteAudioCompressed_.front();
		remoteAudioCompressed_.pop();
		haveRemoteAudioCompressed_ = remoteAudioCompressed_.size() == 0 ? false : true;
		ulRemoteAudioCompressed.unlock();

		audioDecoder_->decode(rawData, remoteAudio_);

		haveRemoteAudio_ = true;
		ulRemoteAudio.unlock();
		haveRemoteAudioCV_.notify_all();
	}
}

void HoloSession::renderAudioLoop()
{
	while (shouldRenderAudio_)
	{
		std::unique_lock<std::mutex> ulRemoteAudioData(remoteAudioMutex_);
		if (!haveRemoteAudio_)
		{
			if (std::cv_status::timeout == haveRemoteAudioCV_.wait_for(ulRemoteAudioData, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
				continue;
		}

		audioRender_->playSoundBuffer(std::move(remoteAudio_));

		haveRemoteAudio_ = false;
		ulRemoteAudioData.unlock();
	}
}