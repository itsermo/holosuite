#pragma once

#include "../common/CommonDefs.hpp"
#include "../holocapture/IHoloCapture.hpp"
#include "../holocodec/IHoloCodec.hpp"
#include "../holofilter/IHoloFilter.hpp"
#include "../holonet/HoloNetSession.hpp"
#include "../holorender/IHoloRender.hpp"

#include <memory>
#include <string>
#include <boost/shared_array.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#define HOLO_SESSION_CV_WAIT_TIMEOUT_MS 500

namespace holo
{

	class HoloSession
	{
	public:
		HoloSession();
		//HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
		//	std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> && codec,
		//	std::shared_ptr<holo::net::HoloNetSession> netsession,
		//	holo::net::HoloNetProtocolHandshake remoteInfo,
		//	std::unique_ptr<holo::render::IHoloRender> && render
		//	);
		HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && encoderRGBAZ,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && decoderRGBAZ,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> && encoderCloud,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> && decoderCloud,
			std::shared_ptr<holo::net::HoloNetSession> netSession, holo::net::HoloNetProtocolHandshake remoteInfo,
			std::unique_ptr<holo::render::IHoloRender> && render
			);
		~HoloSession();

		bool connect();
		bool disconnect();

		bool start();
		void stop();

		bool isRunning();
		bool isConnected();

	private:

		std::string localName_;
		
		holo::net::HoloNetProtocolHandshake remoteInfo_;

		boost::shared_ptr<HoloRGBAZMat> localRGBAZ_;
		boost::shared_ptr<HoloRGBAZMat> remoteRGBAZ_;

		boost::shared_ptr<HoloCloud> localCloud_;
		boost::shared_ptr<HoloCloud> remoteCloud_;

		boost::shared_ptr<std::vector<uchar>> remoteCloudCompressed_;
		boost::shared_ptr<std::vector<uchar>> remoteRGBAZCompressed_;
		boost::shared_ptr<std::vector<uchar>> remoteAudioCompressed_;

		std::thread captureThread_;
		std::thread encodeThread_;
		std::thread decodeThread_;
		std::thread renderThread_;
		std::thread netRecvThread_;

		std::mutex localCloudMutex_;
		std::mutex remoteCloudMutex_;
		std::mutex remoteCloudCompressedMutex_;
		std::mutex remoteAudioCompressedMutex_;

		std::mutex localRGBAZMutex_;
		std::mutex remoteRGBAZMutex_;
		std::mutex remoteRGBAZCompressedMutex_;
		
		std::condition_variable haveLocalCloudCV_;
		std::condition_variable haveLocalRGBAZCV_;
		std::condition_variable haveRemoteCloudCV_;
		std::condition_variable haveRemoteCloudCompressedCV_;
		std::condition_variable haveRemoteAudioCV_;
		std::condition_variable haveRemoteAudioCompressedCV_;
		std::condition_variable haveRemoteRGBAZCV_;
		std::condition_variable haveRemoteRGBAZCompressedCV_;

		std::atomic<bool> haveLocalCloud_;
		std::atomic<bool> haveLocalRGBAZ_;
		std::atomic<bool> haveRemoteCloud_;
		std::atomic<bool> haveRemoteCloudCompressed_;
		std::atomic<bool> haveRemoteAudio_;
		std::atomic<bool> haveRemoteAudioCompressed_;
		std::atomic<bool> haveRemoteRGBAZ_;
		std::atomic<bool> haveRemoteRGBAZCompressed_;

		std::unique_ptr<holo::capture::IHoloCapture> capture_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> cloudEncoder_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> cloudDecoder_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloRGBAZMat>> rgbazEncoder_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloRGBAZMat>> rgbazDecoder_;
		std::shared_ptr<holo::net::HoloNetSession> netSession_;
		std::unique_ptr<holo::render::IHoloRender> render_;

		std::atomic<bool> shouldCapture_;
		std::atomic<bool> shouldEncode_;
		std::atomic<bool> shouldDecode_;
		std::atomic<bool> shouldRender_;
		std::atomic<bool> shouldRecv_;

		std::atomic<bool> isRunning_;

		void captureLoop();
		void decodeLoop();
		void encodeLoop();
		void renderLoop();
		void netRecvLoop();

		holo::capture::WorldConvertCache worldConvertCache_;

		log4cxx::LoggerPtr logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.session");

	};
}