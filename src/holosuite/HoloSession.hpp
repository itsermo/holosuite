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

namespace holo
{

	class HoloSession
	{
	public:
		HoloSession();
		HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
			std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> && codec,
			std::shared_ptr<holo::net::HoloNetSession> netsession,
			holo::net::HoloNetProtocolHandshake remoteInfo,
			std::unique_ptr<holo::render::IHoloRender> && render
			);
		HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && codec,
			std::shared_ptr<holo::net::HoloNetSession> netSession,
			holo::net::HoloNetProtocolHandshake remoteInfo,
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
		
		std::condition_variable haveLocalCloud_;
		std::condition_variable haveLocalRGBAZ_;
		std::condition_variable haveRemoteCloud_;
		std::condition_variable haveRemoteCloudCompressed_;
		std::condition_variable haveRemoteAudio_;
		std::condition_variable haveRemoteAudioCompressed_;
		std::condition_variable haveRemoteRGBAZ_;
		std::condition_variable haveRemoteRGBAZCompressed_;

		std::unique_ptr<holo::capture::IHoloCapture> capture_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> cloudCodec_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloRGBAZMat>> rgbazCodec_;
		std::shared_ptr<holo::net::HoloNetSession> netSession_;
		std::unique_ptr<holo::render::IHoloRender> render_;

		std::atomic<bool> shouldCapture_;
		std::atomic<bool> shouldEncode_;
		std::atomic<bool> shouldDecode_;
		std::atomic<bool> shouldRender_;

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