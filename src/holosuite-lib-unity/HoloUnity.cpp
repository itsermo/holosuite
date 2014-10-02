#include "HoloUnity.h"
#include <holocommon/CommonDefs.hpp>

#include <holocapture/HoloCaptureGenerator.hpp>
#include <holorender/HoloRenderGenerator.hpp>
#include <holocodec/HoloCodecGenerator.hpp>
#include <holonet/HoloNetClient.hpp>
#include <holonet/HoloNetServer.hpp>
#include <holosession/HoloSession.hpp>

holo::net::HoloNetProtocolHandshake g_localInfo = { 0 };
holo::net::HoloNetProtocolHandshake g_remoteInfo = { 0 };
holo::HOLO_SESSION_MODE g_sessionMode;
std::unique_ptr<holo::capture::IHoloCapture> g_videoCapture = nullptr;
std::unique_ptr<holo::capture::IHoloCaptureAudio> g_audioCapture = nullptr;
std::unique_ptr<holo::render::IHoloRenderAudio> g_audioRenderer = nullptr;
std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> g_encoderCloud = nullptr;
std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> g_decoderCloud = nullptr;
std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> g_encoderRGBAZ = nullptr;
std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> g_decoderRGBAZ = nullptr;
std::unique_ptr<holo::codec::IHoloCodec<std::vector<unsigned char>>> g_decoderAudio = nullptr;
std::unique_ptr<holo::codec::IHoloCodec<std::vector<unsigned char>>> g_encoderAudio = nullptr;
std::shared_ptr<holo::net::HoloNetClient> g_client = nullptr;
std::shared_ptr<holo::net::HoloNetServer> g_server = nullptr;
std::unique_ptr<holo::render::IHoloRender> g_renderer = nullptr;
std::unique_ptr<holo::HoloSession> g_clientSession = nullptr;
std::unique_ptr<holo::HoloSession> g_serverSession = nullptr;

extern "C" {

	HOLOUNITY_API void setLocalVisualCallback(cloud_cb_t visualCallback)
	{
		switch (g_sessionMode)
		{
		case holo::HOLO_SESSION_MODE_SERVER:
			g_serverSession->setLocalCloudCallback(visualCallback);
			break;
		case holo::HOLO_SESSION_MODE_CLIENT:
		case holo::HOLO_SESSION_MODE_LOOPBACK:
		case holo::HOLO_SESSION_MODE_DIRECT:
			g_clientSession->setLocalCloudCallback(visualCallback);
			break;
		default:
			break;
		}

	}

	HOLOUNITY_API void setLocalAudioCallback(audio_cb_t audioCallback)
	{
		switch (g_sessionMode)
		{
		case holo::HOLO_SESSION_MODE_SERVER:
			g_serverSession->setLocalAudioCallback(audioCallback);
			break;
		case holo::HOLO_SESSION_MODE_CLIENT:
		case holo::HOLO_SESSION_MODE_LOOPBACK:
		case holo::HOLO_SESSION_MODE_DIRECT:
			g_clientSession->setLocalAudioCallback(audioCallback);
			break;
		default:
			break;
		}
	}

	HOLOUNITY_API void setRemoteVisualCallback(cloud_cb_t visualCallback)
	{
		switch (g_sessionMode)
		{
		case holo::HOLO_SESSION_MODE_SERVER:
			g_serverSession->setRemoteCloudCallback(visualCallback);
			break;
		case holo::HOLO_SESSION_MODE_CLIENT:
		case holo::HOLO_SESSION_MODE_LOOPBACK:
		case holo::HOLO_SESSION_MODE_DIRECT:
			g_clientSession->setRemoteCloudCallback(visualCallback);
		default:
			break;
		}
	}

	HOLOUNITY_API void setRemoteAudioCallback(audio_cb_t audioCallback)
	{
		switch (g_sessionMode)
		{
		case holo::HOLO_SESSION_MODE_SERVER:
			g_serverSession->setRemoteAudioCallback(audioCallback);
			break;
		case holo::HOLO_SESSION_MODE_CLIENT:
		case holo::HOLO_SESSION_MODE_LOOPBACK:
		case holo::HOLO_SESSION_MODE_DIRECT:
			g_clientSession->setRemoteAudioCallback(audioCallback);
			break;
		default:
			break;
		}
	}

	HOLOUNITY_API void initHoloSuite(holo::capture::CAPTURE_TYPE capture,
		holo::capture::CAPTURE_AUDIO_TYPE audioCapture,
		holo::codec::CODEC_TYPE captureEncoder,
		holo::codec::CODEC_TYPE audioEncoder,
		holo::render::RENDER_TYPE visOutput,
		holo::render::RENDER_AUDIO_TYPE audioOutput)
	{
		holo::capture::HoloCaptureInfo captureInfo = { 0 };
		holo::HoloAudioFormat audioFormat = { 0 };

		switch (capture)
		{
		case holo::capture::CAPTURE_TYPE_FILE_PLY:
			//TODO: get PLY file support
			break;
		case holo::capture::CAPTURE_TYPE_FILE_PCD:
			//TODO: get PCD files in
			break;
		case holo::capture::CAPTURE_TYPE_FILE_OBJ:
			//TODO: get OBJ file support
			break;
		case holo::capture::CAPTURE_TYPE_FILE_ONI:
			//g_videoCapture = holo::capture::HoloCaptureGenerator::fromOpenNI2(filePath.string());
			break;
		case holo::capture::CAPTURE_TYPE_OPENNI2:
			g_videoCapture = holo::capture::HoloCaptureGenerator::fromOpenNI2();
			break;
		default:
		case holo::capture::CAPTURE_TYPE_NONE:
			g_videoCapture = nullptr;
			break;
		}

		if (g_videoCapture)
		{
			g_videoCapture->init(0);
			captureInfo = g_videoCapture->getCaptureInfo();
		}

		switch (audioCapture)
		{
		case holo::capture::CAPTURE_AUDIO_TYPE_NONE:
			g_audioCapture = nullptr;
			break;
#ifdef ENABLE_HOLO_AUDIO
		case holo::capture::CAPTURE_AUDIO_TYPE_PORTAUDIO:
			g_audioCapture = holo::capture::HoloCaptureGenerator::fromPortaudio();
			break;
#endif
		default:
			break;
		}

		if (g_audioCapture)
		{
			g_audioCapture->init(0);
			audioFormat = g_audioCapture->getAudioFormat();
		}

		switch (captureEncoder)
		{
		case holo::codec::CODEC_TYPE_PASSTHROUGH_CLOUD:
			g_encoderCloud = holo::codec::HoloCodecGenerator::fromPCLPassthrough();
			break;
		case holo::codec::CODEC_TYPE_PASSTHROUGH_RGBAZ:
			//TODO: implement passthrough RGBAZ
			break;
		case holo::codec::CODEC_TYPE_OCTREE:
			g_encoderCloud = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression();
			break;
		case holo::codec::CODEC_TYPE_H264:
			g_encoderRGBAZ = holo::codec::HoloCodecGenerator::fromH264();
			break;
		case holo::codec::CODEC_TYPE_NONE:
		default:
			g_encoderCloud = nullptr;
			g_encoderRGBAZ = nullptr;
			break;
		}

		switch (audioEncoder)
		{
#ifdef ENABLE_HOLO_AUDIO
		case holo::codec::CODEC_TYPE_OPUS:
			g_encoderAudio = holo::codec::HoloCodecGenerator::fromOpus();
			break;
#endif
		case holo::codec::CODEC_TYPE_NONE:
		default:
			g_encoderAudio = nullptr;
			break;
		}

		switch (visOutput)
		{
		case holo::render::RENDER_TYPE_VIS2D:
			//TODO: implement 2D VIS
			break;
		case holo::render::RENDER_TYPE_VIS3D:
			g_renderer = holo::render::HoloRenderGenerator::fromPCLVisualizer();
			break;
#ifdef ENABLE_HOLO_DSCP2
		case holo::render::RENDER_TYPE_DSCP_MKII:
			g_renderer = holo::render::HoloRenderGenerator::fromDSCP2();
			break;
#endif
		case holo::render::RENDER_TYPE_DSCP_MKIV:
			//TODO: implement mk iv dscp algo
			break;
		case holo::render::RENDER_TYPE_NONE:
		default:
			g_renderer = nullptr;
			break;
		}

		switch (audioOutput)
		{
#ifdef ENABLE_HOLO_AUDIO
		case holo::render::RENDER_AUDIO_TYPE_PORTAUDIO:
			g_audioRenderer = holo::render::HoloRenderGenerator::fromPortaudio();
			break;
#endif
		case holo::render::RENDER_TYPE_NONE:
		default:
			g_audioRenderer = nullptr;
			break;
		}

		//g_localInfo.magicNumber = 655321;
		//g_localInfo.clientName = "UNITY";
		//g_localInfo.protocolVersion = 1;
		strcpy((char*)g_localInfo.clientName, "UNITY");
		g_localInfo.videoCodecType = captureEncoder;
		g_localInfo.audioCodecType = audioEncoder;
		g_localInfo.rgbazWidth = captureInfo.zWidth;
		

	}

	HOLOUNITY_API void deinitHoloSuite()
	{

	}

	HOLOUNITY_API void initSession(holo::HOLO_SESSION_MODE sessionMode, const char * address)
	{
		if (sessionMode == holo::HOLO_SESSION_MODE_DIRECT)
		{
			g_clientSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(
				std::move(g_videoCapture),
				std::move(g_audioCapture),
				nullptr,
				nullptr,
				nullptr,
				nullptr,
				nullptr,
				nullptr,
				std::move(g_renderer),
				std::move(g_audioRenderer),
				nullptr,
				g_localInfo
				));
		}

	}

	HOLOUNITY_API void deInitSession()
	{
		//g_clientSession->disconnect();
	}

	HOLOUNITY_API void startSession()
	{
		g_clientSession->start();
	}

	HOLOUNITY_API void stopSession()
	{
		g_clientSession->stop();
	}
}