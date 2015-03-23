#pragma once

#include "IHoloInputDevice.hpp"

#ifdef ENABLE_HOLO_LEAPSDK
#include "HoloInputDeviceLeapSDK.hpp"
#endif

#include <memory>

namespace holo
{
	namespace input
	{
		class HoloInputDeviceGenerator
		{
		public:
			HoloInputDeviceGenerator();
			~HoloInputDeviceGenerator();

#ifdef ENABLE_HOLO_LEAPSDK
			static std::unique_ptr<IHoloInputDevice> fromLeapSDK()
			{
				return std::unique_ptr<IHoloInputDevice>(new HoloInputDeviceLeapSDK);
			}
#endif
		};
	}
}