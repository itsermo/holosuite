#define HOLOUNITY_API __declspec(dllexport)
#include <holocommon/CommonDefs.hpp>

extern "C" {

	typedef void(__stdcall * rgbaz_cb_t)(unsigned char*, unsigned short*, const void*);
	typedef void(__stdcall * cloud_cb_t)(void*, int);
	typedef void(__stdcall * mesh_cb_t)(unsigned char*, int, const void*, int);
	typedef void(__stdcall * audio_cb_t)(unsigned char*, int);

	HOLOUNITY_API void setLocalVisualCallback(cloud_cb_t visualCallback);
	HOLOUNITY_API void setRemoteVisualCallback(cloud_cb_t visualCallback);
	HOLOUNITY_API void setLocalAudioCallback(audio_cb_t audioCallback);
	HOLOUNITY_API void setRemoteAudioCallback(audio_cb_t audioCallback);

	HOLOUNITY_API void initHoloSuite(holo::capture::CAPTURE_TYPE capture,
		holo::capture::CAPTURE_AUDIO_TYPE audioCapture,
		holo::codec::CODEC_TYPE captureEncoder, holo::codec::CODEC_TYPE audioCaptureEncoder, holo::render::RENDER_TYPE visOutput, holo::render::RENDER_AUDIO_TYPE audioOutput);
	HOLOUNITY_API void deinitHoloSuite();

	HOLOUNITY_API void startServer();
	HOLOUNITY_API void connectToServer(unsigned char * address);

	HOLOUNITY_API void initSession(holo::HOLO_SESSION_MODE sessionMode, const char * address);
	HOLOUNITY_API void deInitSession();

	HOLOUNITY_API void startSession();
	HOLOUNITY_API void stopSession();
}