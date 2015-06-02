#pragma once

#include <holocommon/CommonDefs.hpp>
#include <holocapture/IHoloCapture.hpp>
#include <holocodec/IHoloCodec.hpp>
#include <holofilter/IHoloFilter.hpp>
#include <holonet/HoloNetSession.hpp>
#include <holorender/IHoloRender.hpp>
#include <holocapture/IHoloCaptureAudio.hpp>
#include <holorender/IHoloRenderAudio.hpp>
#include <holoinput/IHoloInputDevice.hpp>
#include <holorender/HoloRenderObjectTracker.hpp>


#include <memory>
#include <string>
#include <boost/shared_array.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#ifndef WIN32
#define __stdcall
#endif

typedef void(__stdcall * RGBAZCallback)(unsigned char*, unsigned short*, const void*);
typedef void(__stdcall * CloudCallback)(void*, int);
typedef void(__stdcall * MeshCallback)(unsigned char*, int, const void*, int);
typedef void(__stdcall * AudioCallback)(unsigned char*, int);

namespace holo
{
	
	class HoloSession
	{
	public:

		enum HoloSessionCallbackType
		{
			HOLO_SESSION_CALLBACK_LOCAL_RGBAZ,
			HOLO_SESSION_CALLBACK_LOCAL_CLOUD,
			HOLO_SESSION_CALLBACK_LOCAL_MESH,
			HOLO_SESSION_CALLBACK_LOCAL_AUDIO,
			HOLO_SESSION_CALLBACK_REMOTE_RGBAZ,
			HOLO_SESSION_CALLBACK_REMOTE_CLOUD,
			HOLO_SESSION_CALLBACK_REMOTE_MESH,
			HOLO_SESSION_CALLBACK_REMOTE_AUDIO,
		};

		HoloSession();
		HoloSession(std::unique_ptr<holo::capture::IHoloCapture> && capture,
			std::unique_ptr<holo::capture::IHoloCaptureAudio> && audioCapture,
			std::unique_ptr<holo::input::IHoloInputDevice> && inputDevice,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && encoderRGBAZ,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> && decoderRGBAZ,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> && encoderCloud,
			std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> && decoderCloud,
			std::unique_ptr<holo::codec::IHoloCodec<std::vector<uchar>>> && audioEncoder,
			std::unique_ptr<holo::codec::IHoloCodec<std::vector<uchar>>> && audioDecoder,
			boost::shared_ptr<holo::render::IHoloRender> && render,
			std::unique_ptr<holo::render::IHoloRenderAudio> && audioRender,
			std::shared_ptr<holo::net::HoloNetSession> netSession, holo::net::HoloNetProtocolHandshake remoteInfo,
			bool enableVisualFeedback
			);
		~HoloSession();

		bool connect();
		bool disconnect();

		bool start();
		void stop();

		bool isRunning();
		bool isConnected();

		void setLocalCloudCallback(CloudCallback cloudCallback);
		void setLocalRGBAZCallback(RGBAZCallback cloudCallback);
		void setLocalAudioCallback(AudioCallback cloudCallback);

		void setRemoteCloudCallback(CloudCallback cloudCallback);
		void setRemoteRGBAZCallback(RGBAZCallback cloudCallback);
		void setRemoteAudioCallback(AudioCallback cloudCallback);

	private:
		
		//void stop(std::thread::id callingThread);

		bool enableVisualFeedback_;

		std::string localName_;
		
		holo::net::HoloNetProtocolHandshake remoteInfo_;

		boost::shared_ptr<HoloRGBAZMat> localRGBAZ_;
		boost::shared_ptr<HoloRGBAZMat> remoteRGBAZ_;

		boost::shared_ptr<HoloCloud> localCloud_;
		boost::shared_ptr<HoloCloud> remoteCloud_;

		std::queue<boost::shared_ptr<std::vector<uchar>>> remoteCloudCompressed_;
		std::queue<boost::shared_ptr<std::vector<uchar>>> remoteRGBAZCompressed_;

		boost::shared_ptr<std::vector<uchar>> localAudio_;
		boost::shared_ptr<std::vector<uchar>> remoteAudio_;
		std::queue<boost::shared_ptr<std::vector<uchar>>> remoteAudioCompressed_;

		std::thread captureThread_;
		std::thread captureAudioThread_;
		std::thread interactionThread_;

		std::thread encodeThread_;
		std::thread encodeAudioThread_;

		std::thread decodeThread_;
		std::thread decodeAudioThread_;

		std::thread renderThread_;
		std::thread renderAudioThread_;

		std::thread netRecvThread_;

		std::thread objectTrackerThread_;

		std::thread localCloudCallbackThread_;
		std::thread remoteCloudCallbackThread_;

		std::thread localRGBAZCallbackThread_;
		std::thread remoteRGBAZCallbackThread_;

		std::thread localAudioCallbackThread_;
		std::thread remoteAudioCallbackThread_;

		std::mutex localCloudMutex_;
		std::mutex remoteCloudMutex_;
		std::mutex remoteCloudCompressedMutex_;

		std::mutex localRGBAZMutex_;
		std::mutex remoteRGBAZMutex_;
		std::mutex remoteRGBAZCompressedMutex_;

		std::mutex localAudioMutex_;
		std::mutex remoteAudioMutex_;
		std::mutex remoteAudioCompressedMutex_;
		
		std::mutex remoteObjectDataMutex_;
		std::mutex localObjectDataMutex_;

		std::mutex localInteractionDataMutex_;
		std::mutex remoteInteractionDataMutex_;

		std::mutex stoppingMutex_;

		std::condition_variable haveLocalCloudCV_;
		std::condition_variable haveLocalRGBAZCV_;
		std::condition_variable haveLocalAudioCV_;
		std::condition_variable haveRemoteCloudCV_;
		std::condition_variable haveLocalInteractionDataCV_;
		std::condition_variable haveRemoteCloudCompressedCV_;
		std::condition_variable haveRemoteAudioCV_;
		std::condition_variable haveRemoteAudioCompressedCV_;
		std::condition_variable haveRemoteRGBAZCV_;
		std::condition_variable haveRemoteRGBAZCompressedCV_;
		std::condition_variable haveRemoteObjectDataCV_;
		std::condition_variable haveRemoteInteractionDataCV_;

		std::atomic<bool> haveLocalCloud_;
		std::atomic<bool> haveLocalAudio_;
		std::atomic<bool> haveLocalRGBAZ_;
		std::atomic<bool> haveLocalRemoteObjectData_;
		std::atomic<bool> haveRemoteCloud_;
		std::atomic<bool> haveRemoteCloudCompressed_;
		std::atomic<bool> haveRemoteAudio_;
		std::atomic<bool> haveRemoteAudioCompressed_;
		std::atomic<bool> haveRemoteRGBAZ_;
		std::atomic<bool> haveRemoteRGBAZCompressed_;
		std::atomic<bool> haveRemoteObjectData_;
		std::atomic<bool> haveLocalInteractionData_;
		std::atomic<bool> haveRemoteInteractionData_;

		std::unique_ptr<holo::capture::IHoloCapture> capture_;
		std::unique_ptr<holo::capture::IHoloCaptureAudio> audioCapture_;
		std::unique_ptr<holo::input::IHoloInputDevice> inputDevice_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> cloudEncoder_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloCloud>> cloudDecoder_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloRGBAZMat>> rgbazEncoder_;
		std::unique_ptr<holo::codec::IHoloCodec<HoloRGBAZMat>> rgbazDecoder_;
		std::unique_ptr<holo::codec::IHoloCodec<std::vector<unsigned char>>> audioDecoder_;
		std::unique_ptr<holo::codec::IHoloCodec<std::vector<unsigned char>>> audioEncoder_;
		std::shared_ptr<holo::net::HoloNetSession> netSession_;
		boost::shared_ptr<holo::render::IHoloRender> render_;
		std::unique_ptr<holo::render::IHoloRenderAudio> audioRender_;
		boost::shared_ptr<holo::render::HoloRenderObjectTracker> objectTracker_;

		std::list<std::tuple<std::string, holo::render::HoloTransform>> remoteObjectStateData_;
		std::list<std::tuple<std::string, holo::render::HoloTransform>> localObjectStateData_;

		std::list<holo::input::HoloInputData> localInteractionData_;
		std::list<holo::input::HoloInputData> remoteInteractionData_;

		CloudCallback localCloudCallback_;
		RGBAZCallback localRGBAZCallback_;
		AudioCallback localAudioCallback_;

		CloudCallback remoteCloudCallback_;
		RGBAZCallback remoteRGBAZCallback_;
		AudioCallback remoteAudioCallback_;

		std::atomic<bool> shouldCapture_;
		std::atomic<bool> shouldCaptureAudio_;
		std::atomic<bool> shouldInteract_;

		std::atomic<bool> shouldEncode_;
		std::atomic<bool> shouldEncodeAudio_;

		std::atomic<bool> shouldDecode_;
		std::atomic<bool> shouldDecodeAudio_;

		std::atomic<bool> shouldRender_;
		std::atomic<bool> shouldRenderAudio_;

		std::atomic<bool> shouldRecv_;

		std::atomic<bool> shouldCallback_;

		std::atomic<bool> shouldTrackObjects_;

		std::atomic<bool> isRunning_;

		void captureLoop();
		void captureAudioLoop();
		void interactionLoop();

		void decodeLoop();
		void decodeAudioLoop();

		void encodeLoop();
		void encodeAudioLoop();

		void renderLoop();
		void renderAudioLoop();

		void netRecvLoop();
		
		void objectTrackerLoop();
		void sendAllObjects();

		void callbackLoop(HoloSessionCallbackType type);

		holo::capture::WorldConvertCache worldConvertCache_;

		log4cxx::LoggerPtr logger_;

	};
}