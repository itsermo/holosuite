#ifdef ENABLE_HOLO_LEAPSDK

#include "HoloInputDeviceLeapSDK.hpp"

using namespace holo;
using namespace holo::input;
using namespace Leap;

HoloInputDeviceLeapSDK::HoloInputDeviceLeapSDK() :
	isInit_(false),
	inputData_()
{

}

HoloInputDeviceLeapSDK::~HoloInputDeviceLeapSDK()
{

}

bool HoloInputDeviceLeapSDK::init()
{
	controller_.enableGesture(Leap::Gesture::Type::TYPE_SCREEN_TAP);
	controller_.enableGesture(Leap::Gesture::Type::TYPE_CIRCLE);
	controller_.addListener(*this);

	controller_.config().setFloat("Gesture.Circle.MinArc", 1.8*M_PI);
	//controller_.config().save();

	std::unique_lock<std::mutex> initLG(initMutex_);
	
	if (std::cv_status::timeout == hasInitCV_.wait_for(initLG, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
	{
		isInit_.store(false);
		return isInit_.load();
	}

	initLG.unlock();

	isInit_ = true;

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

bool HoloInputDeviceLeapSDK::getInputData(std::list<HoloInputData> &data)
{
	if (isInit_)
	{
		std::unique_lock<std::mutex> inDataLock(inputDataMutex_);
		while (std::cv_status::timeout == haveInputDataCV_.wait_for(inDataLock, std::chrono::milliseconds(HOLO_SESSION_CV_WAIT_TIMEOUT_MS)))
		{
			if (!isInit_)
			{
				return false;
			}
		}
		
		data = inputData_;
		inputData_.clear();

		return true;
	}
	else
	{
		return false;
	}
}

void HoloInputDeviceLeapSDK::onInit(const Leap::Controller& controller)
{
	hasInitCV_.notify_one();
}

void HoloInputDeviceLeapSDK::onConnect(const Leap::Controller& controller)
{
	hasInitCV_.notify_one();
}

void HoloInputDeviceLeapSDK::onDisconnect(const Leap::Controller& controller)
{
	deinit();
}

void HoloInputDeviceLeapSDK::onExit(const Leap::Controller& controller)
{

}

void HoloInputDeviceLeapSDK::onFrame(const Leap::Controller& controller)
{

	HoloInputData inputData = {};
	inputData.leftHand = {};
	//inputData_.rightHand = {};

	const Frame frame = controller.frame();
	HandList hands = frame.hands();
	for (HandList::const_iterator hl = hands.begin(); hl != hands.end(); ++hl) {
		// Get the first hand
		const Hand hand = *hl;
		HoloHand *holoHand = hand.isLeft() ? &inputData.leftHand : &inputData.rightHand;

		if (hand.isLeft())
			inputData.haveLeftHand = true;
		else
			inputData.haveRightHand = true;

		// Get the hand's normal vector and direction
		const Vector normal = hand.palmNormal();
		const Vector position = hand.palmPosition();

		holoHand->palmNormal.x = normal.pitch();
		holoHand->palmNormal.y = normal.roll();
		holoHand->palmNormal.z = normal.yaw();

		holoHand->palmPosition.x = position.x * 0.001f;
		holoHand->palmPosition.y = position.y * 0.001f;
		holoHand->palmPosition.z = position.z * 0.001f;
		
		hand.basis().toArray4x4(holoHand->palmOrientation);

		holoHand->grabStrength = hand.grabStrength();
		holoHand->pinchStrength = hand.pinchStrength();

		// Get fingers
		const FingerList fingers = hand.fingers();
		for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl) {
			const Finger finger = *fl;
			HoloFinger *holoFinger = nullptr;

			switch (finger.type())
			{
			case Leap::Finger::TYPE_THUMB:
				holoFinger = &holoHand->thumb;
				break;
			case Leap::Finger::TYPE_INDEX:
				holoFinger = &holoHand->index;
				break;
			case Leap::Finger::TYPE_MIDDLE:
				holoFinger = &holoHand->middle;
				break;
			case Leap::Finger::TYPE_RING:
				holoFinger = &holoHand->ring;
				break;
			case Leap::Finger::TYPE_PINKY:
				holoFinger = &holoHand->pinky;
				break;
			default:
				break;
			}

			// Get finger bones
			for (int b = 0; b < 4; ++b) {
				Bone::Type boneType = static_cast<Bone::Type>(b);
				Bone bone = finger.bone(boneType);

				holoFinger->bones[b].center.x = bone.center().x * 0.001f;
				holoFinger->bones[b].center.y = bone.center().y * 0.001f;
				holoFinger->bones[b].center.z = bone.center().z * 0.001f;

				holoFinger->bones[b].direction.x = bone.direction().x * 0.001f;
				holoFinger->bones[b].direction.y = bone.direction().y * 0.001f;
				holoFinger->bones[b].direction.z = bone.direction().z * 0.001f;
			}
		}

	}

	// Get gestures
	const GestureList gestures = frame.gestures();
	for (int g = 0; g < gestures.count(); ++g) {
		Gesture gesture = gestures[g];

		HoloHand *holoHand = gesture.hands()[0].isLeft() ? &inputData.leftHand : &inputData.rightHand;

		if (gesture.hands()[0].isLeft())
			inputData.haveLeftHand = true;
		else
			inputData.haveRightHand = true;

		switch (gesture.type()) {
		case Gesture::TYPE_CIRCLE:
		{
			if (gesture.state() == Leap::Gesture::STATE_STOP)
				holoHand->gesture = holo::input::GESTURE_TYPE_CIRCLE;
			break;
		}
		case Gesture::TYPE_SWIPE:
		{	
			holoHand->gesture = holo::input::GESTURE_TYPE_SWIPE;
			break;
		}
		case Gesture::TYPE_KEY_TAP:
		{
			holoHand->gesture = holo::input::GESTURE_TYPE_KEY_TAP;
			break;
		}
		case Gesture::TYPE_SCREEN_TAP:
		{
			holoHand->gesture = holo::input::GESTURE_TYPE_SCREEN_TAP;
			break;
		}
		default:
			std::cout << std::string(2, ' ') << "Unknown gesture type." << std::endl;
			break;
		}
	}

	std::unique_lock<std::mutex> inputDataLock(inputDataMutex_);

	//don't overflow buffer
	if (inputData_.size() > 120)
		inputData_.clear();

	inputData_.push_back(inputData);
	inputDataLock.unlock();
	haveInputDataCV_.notify_all();
}

void HoloInputDeviceLeapSDK::onFocusGained(const Leap::Controller& controller)
{
	// Don't care
}

void HoloInputDeviceLeapSDK::onFocusLost(const Leap::Controller& controller)
{
	// Don't care
}

void HoloInputDeviceLeapSDK::onDeviceChange(const Leap::Controller& controller)
{
	// Don't care
}

void HoloInputDeviceLeapSDK::onServiceConnect(const Leap::Controller& controller)
{
	// Don't care
}

void HoloInputDeviceLeapSDK::onServiceDisconnect(const Leap::Controller& controller)
{
	isInit_.store(false);
}

#endif