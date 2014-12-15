#ifdef ENABLE_HOLO_LEAPSDK

#include "HoloInputDeviceLeapSDK.hpp"

using namespace holo;
using namespace holo::input;

HoloInputDeviceLeapSDK::HoloInputDeviceLeapSDK() :
	isInit_(false)
{

}

HoloInputDeviceLeapSDK::~HoloInputDeviceLeapSDK()
{

}

bool HoloInputDeviceLeapSDK::init()
{
	controller_.addListener(*this);

	std::unique_lock<std::mutex> initLG(initMutex_);
	
	if (std::cv_status::timeout == hasInitCV_.wait_for(initLG, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
	{
		isInit_.store(false);
		return isInit_.load();
	}

	initLG.unlock();

	return true;
}

void HoloInputDeviceLeapSDK::deinit()
{
	isInit_.store(false);
	controller_.removeListener(*this);
}

bool HoloInputDeviceLeapSDK::isInit()
{
	return isInit_.load();
}

bool HoloInputDeviceLeapSDK::getInputData(void * data)
{
	if (isInit_)
	{

		return true;
	}
	else
		return false;
}

void HoloInputDeviceLeapSDK::onInit(const Leap::Controller&)
{
	hasInitCV_.notify_one();
}

void HoloInputDeviceLeapSDK::onConnect(const Leap::Controller&)
{
	hasInitCV_.notify_one();
}

void HoloInputDeviceLeapSDK::onDisconnect(const Leap::Controller&)
{
	deinit();
}

void HoloInputDeviceLeapSDK::onExit(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onFrame(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onFocusGained(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onFocusLost(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onDeviceChange(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onServiceConnect(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onServiceDisconnect(const Leap::Controller&)
{

}

#endif