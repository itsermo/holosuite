#pragma once

#ifdef ENABLE_HOLO_LEAPSDK

#include <holoinput/IHoloInputDevice.hpp>
#include <Leap.h>

#include <atomic>
#include <mutex>
#include <condition_variable>

namespace holo
{
	namespace input
	{
		class HoloInputDeviceLeapSDK : public IHoloInputDevice, public Leap::Listener
		{
		public:
			HoloInputDeviceLeapSDK();
			~HoloInputDeviceLeapSDK();
			bool init();
			void deinit();
			bool isInit();

			bool getInputData(InputData *& data);

			void onInit(const Leap::Controller&);
			void onConnect(const Leap::Controller&);
			void onDisconnect(const Leap::Controller&);
			void onExit(const Leap::Controller&);
			void onFrame(const Leap::Controller&);
			void onFocusGained(const Leap::Controller&);
			void onFocusLost(const Leap::Controller&);
			void onDeviceChange(const Leap::Controller&);
			void onServiceConnect(const Leap::Controller&);
			void onServiceDisconnect(const Leap::Controller&);

		private:

			std::atomic<bool> isInit_;

			std::condition_variable hasInitCV_;
			std::mutex initMutex_;

			Leap::Controller controller_;

			InputData inputData_;
			std::mutex inputDataMutex_;
			std::condition_variable haveInputDataCV_;

		};
	}
}

#endif