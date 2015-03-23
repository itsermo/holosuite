#pragma once

#include <holocommon/CommonDefs.hpp>

namespace holo
{
	namespace input
	{
		class IHoloInputDevice
		{
		public:
			virtual ~IHoloInputDevice() = 0;
			virtual bool init() = 0;
			virtual void deinit() = 0;
			virtual bool isInit() = 0;

			virtual bool getInputData(InputData *& data) = 0;
		};

		inline IHoloInputDevice::~IHoloInputDevice() {}
	}
}