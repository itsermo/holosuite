#include "HoloCaptureOpenNI2Listener.hpp"

#include "../holoutils/HoloUtils.hpp"

using namespace holo;
using namespace holo::capture;

HoloCaptureOpenNI2Listener::HoloCaptureOpenNI2Listener() : haveNewColorFrame_(false), haveNewDepthFrame_(false)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.openni2");
}

HoloCaptureOpenNI2Listener::~HoloCaptureOpenNI2Listener()
{

}

void HoloCaptureOpenNI2Listener::getColorFrame(cv::Mat& colorOut)
{
	auto frameLock = std::unique_lock<std::mutex>(colorFrameMutex_);
	if (!haveNewColorFrame_)
	if (std::cv_status::timeout == colorFrameCV_.wait_for(frameLock, std::chrono::milliseconds(HOLO_CAPTURE_OPENNI2_MUTEX_TIMEOUT_MS)))
	{
		LOG4CXX_ERROR(logger_, "Could not process color frame due to mutex lock timeout");
		return;
	}

	auto rgbImage = cv::Mat(cv::Size(colorFrame_.getWidth(), colorFrame_.getHeight()), CV_8UC3, (void*)colorFrame_.getData(), colorFrame_.getStrideInBytes());
	holo::utils::ConvertRGBToRGBA(rgbImage, colorOut);

	haveNewColorFrame_ = false;
	frameLock.unlock();
}


void HoloCaptureOpenNI2Listener::getDepthFrame(cv::Mat& depthOut)
{
	auto frameLock = std::unique_lock<std::mutex>(depthFrameMutex_);
	if (!haveNewDepthFrame_)
	if (std::cv_status::timeout == depthFrameCV_.wait_for(frameLock, std::chrono::milliseconds(HOLO_CAPTURE_OPENNI2_MUTEX_TIMEOUT_MS)))
	{
		LOG4CXX_ERROR(logger_, "Could not process the depth frame due to mutex lock timeout");
		return;
	}

	depthOut = cv::Mat(depthFrame_.getHeight(), depthFrame_.getWidth(), CV_16UC1);
	memcpy((void*)depthOut.datastart, depthFrame_.getData(), depthFrame_.getDataSize());

	haveNewDepthFrame_ = false;
	frameLock.unlock();
}


void HoloCaptureOpenNI2Listener::onNewFrame(openni::VideoStream& stream)
{
	openni::SensorType type = stream.getSensorInfo().getSensorType();
	switch (type)
	{
	case openni::SENSOR_COLOR:
		{
			auto colorLock = std::unique_lock<std::mutex>(colorFrameMutex_);
			stream.readFrame(&colorFrame_);
			haveNewColorFrame_ = true;
			colorLock.unlock();
			colorFrameCV_.notify_all();
		}
		break;
	case openni::SENSOR_DEPTH:
		{
			auto depthLock = std::unique_lock<std::mutex>(depthFrameMutex_);
			stream.readFrame(&depthFrame_);
			haveNewDepthFrame_ = true;
			depthLock.unlock();
			colorFrameCV_.notify_all();
		}
		break;
	default:
		LOG4CXX_WARN(logger_, "Got a a new frame of unimplemented type");
		break;
	}
}
