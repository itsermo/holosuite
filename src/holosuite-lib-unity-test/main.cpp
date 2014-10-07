#include "../holosuite-lib-unity/HoloUnity.h"
#include <holocommon/CommonDefs.hpp>
#include <thread>

void localCloudCallback(void*, int);

int main(int argc, char *argv[])
{

	initHoloSuite(holo::capture::CAPTURE_TYPE_OPENNI2, holo::capture::CAPTURE_AUDIO_TYPE_NONE, holo::codec::CODEC_TYPE_NONE, holo::codec::CODEC_TYPE_NONE, holo::render::RENDER_TYPE_NONE, holo::render::RENDER_AUDIO_TYPE_NONE);
	//deinitHoloSuite();



	initSession(holo::HOLO_SESSION_MODE_DIRECT, "NOTHING");
	setRemoteVisualCallback(localCloudCallback);

	startSession();

	std::this_thread::sleep_for(std::chrono::milliseconds(4000));


	deInitSession();

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(2000));
	}
}

void localCloudCallback(void* points, int numPoints)
{
	holo::HoloPoint3D * point = (holo::HoloPoint3D*)points;

	for (int i = 0; i < numPoints; i++)
	{
		if (point->r > 0)
		{
			printf("WOW");
		}
		point++;
	}
}