#pragma once

#ifdef ENABLE_HOLO_LEAPSDK

#include <holoinput/IHoloInputDevice.hpp>
#include <Leap.h>

namespace holo
{
	namespace input
	{
		class HoloInputDeviceLeapSDK : IHoloInputDevice
		{
		public:
			HoloInputDeviceLeapSDK();
			~HoloInputDeviceLeapSDK();
			bool init();
			void deinit();
			bool isInit();

			bool getInputData(void * data);
		};
	}
}

#endif