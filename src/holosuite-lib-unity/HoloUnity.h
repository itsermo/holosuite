#define HOLOUNITY_API __declspec(dllexport)
#include <holocommon/CommonDefs.hpp>

HOLOUNITY_API void setLocalVisualCallback(CloudCallback visualCallback);
HOLOUNITY_API void setRemoteVisualCallback(CloudCallback visualCallback);
HOLOUNITY_API void setLocalAudioCallback(AudioCallback audioCallback);
HOLOUNITY_API void setRemoteAudioCallback(AudioCallback audioCallback);

HOLOUNITY_API void initHoloSuite(holo::capture::CAPTURE_TYPE capture,
	holo::capture::CAPTURE_AUDIO_TYPE audioCapture,
	holo::codec::CODEC_TYPE captureEncoder, holo::codec::CODEC_TYPE audioCaptureEncoder, holo::render::RENDER_TYPE visOutput, holo::render::RENDER_AUDIO_TYPE audioOutput);
HOLOUNITY_API void deinitHoloSuite();

HOLOUNITY_API void startServer();
HOLOUNITY_API void connectToServer(unsigned char * address);

HOLOUNITY_API void initSession(holo::HOLO_SESSION_MODE sessionMode, unsigned char * address);
HOLOUNITY_API void deInitSession();

