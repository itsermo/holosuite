#include "../holosuite-lib-unity/HoloUnity.h"
#include <holocommon/CommonDefs.hpp>

void localCloudCallback(void*, int);

int main(int argc, char *argv[])
{
	initHoloSuite(holo::capture::CAPTURE_TYPE_OPENNI2, holo::capture::CAPTURE_AUDIO_TYPE_NONE, holo::codec::CODEC_TYPE_NONE, holo::codec::CODEC_TYPE_NONE, holo::render::RENDER_TYPE_NONE, holo::render::RENDER_AUDIO_TYPE_NONE);
	
	initSession(holo::HOLO_SESSION_MODE_DIRECT, "NOTHING");

	setLocalVisualCallback(localCloudCallback);

	startSession();
}

void localCloudCallback(void* points, int numPoints)
{
	for (int i = 0; i < numPoints; i++)
	{
		holo::HoloPoint3D * point = (holo::HoloPoint3D*)points;
		point++;
	}
}