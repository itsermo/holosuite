#ifdef ENABLE_HOLO_LEAPSDK

#include "HoloInputDeviceLeapSDK.hpp"

using namespace holo;
using namespace holo::input;

HoloInputDeviceLeapSDK::HoloInputDeviceLeapSDK()
{

}

HoloInputDeviceLeapSDK::~HoloInputDeviceLeapSDK()
{

}

bool HoloInputDeviceLeapSDK::init()
{
	controller_.addListener(*this);

	std::unique_lock<std::mutex> initLG(initMutex_);
	
	hasInitCV_.wait(initLG);

	initLG.unlock();

	return true;
}

void HoloInputDeviceLeapSDK::deinit()
{
	controller_.removeListener(*this);
}

bool HoloInputDeviceLeapSDK::isInit()
{
	return false;
}

bool HoloInputDeviceLeapSDK::getInputData(void * data)
{

	return false;
}

void HoloInputDeviceLeapSDK::onInit(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onConnect(const Leap::Controller&)
{

}

void HoloInputDeviceLeapSDK::onDisconnect(const Leap::Controller&)
{

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