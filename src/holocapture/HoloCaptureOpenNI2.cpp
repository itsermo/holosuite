#include "HoloCaptureOpenNI2.hpp"
#include <future>

using namespace holo;
using namespace holo::capture;
using namespace openni;

HoloCaptureOpenNI2::HoloCaptureOpenNI2() :
rgbWidth_(HOLO_CAPTURE_DEFAULT_RGB_WIDTH),
rgbHeight_(HOLO_CAPTURE_DEFAULT_RGB_HEIGHT),
rgbFPS_(HOLO_CAPTURE_DEFAULT_RGB_FPS),
zWidth_(HOLO_CAPTURE_DEFAULT_Z_WIDTH),
zHeight_(HOLO_CAPTURE_DEFAULT_Z_HEIGHT),
zFPS_(HOLO_CAPTURE_DEFAULT_Z_FPS),
isOpen_(false),
isOpenNI2Init_(false), worldConvertCache_()
{
	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object instantiated");
}

HoloCaptureOpenNI2::HoloCaptureOpenNI2(std::string filePath) :
rgbWidth_(HOLO_CAPTURE_DEFAULT_RGB_WIDTH),
rgbHeight_(HOLO_CAPTURE_DEFAULT_RGB_HEIGHT),
rgbFPS_(HOLO_CAPTURE_DEFAULT_RGB_FPS),
zWidth_(HOLO_CAPTURE_DEFAULT_Z_WIDTH),
zHeight_(HOLO_CAPTURE_DEFAULT_Z_HEIGHT),
zFPS_(HOLO_CAPTURE_DEFAULT_Z_FPS),
isOpen_(false),
isOpenNI2Init_(false),
filePath_(filePath), worldConvertCache_()
{
	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object instantiated");
}

HoloCaptureOpenNI2::HoloCaptureOpenNI2(unsigned int rgbWidth,
	unsigned int rgbHeight,
	float rgbFPS,
	unsigned int zWidth,
	unsigned int zHeight,
	float zFPS) :
rgbWidth_(rgbWidth),
rgbHeight_(rgbHeight),
rgbFPS_(rgbFPS),
zWidth_(zWidth),
zHeight_(zHeight),
zFPS_(zFPS),
isOpen_(false),
isOpenNI2Init_(false), worldConvertCache_()
{
	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object instantiated");
}

HoloCaptureOpenNI2::~HoloCaptureOpenNI2()
{
	deinit();
	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object destroyed");
}

bool HoloCaptureOpenNI2::init(int which)
{
	LOG4CXX_INFO(logger_, "Initializing OpenNI2 capture device")
		LOG4CXX_DEBUG(logger_, "Using capture mode: \n"
		<< "Camera index: " << which << std::endl
		<< "RGB Width: " << rgbWidth_ << std::endl
		<< "RGB Height: " << rgbHeight_ << std::endl
		<< "RGB Framerate: " << rgbFPS_ << std::endl
		<< "Z Width: " << zWidth_ << std::endl
		<< "Z Height: " << zHeight_ << std::endl
		<< "Z FPS: " << zFPS_ << std::endl);

	Status rc = Status::STATUS_OK;

	initOpenNI2();

	openni::Array<openni::DeviceInfo> deviceArray;
	OpenNI::enumerateDevices(&deviceArray);

	if ((deviceArray.getSize() < which || which < 0) && filePath_.empty())
	{
		logger_->error("Depth camera device selection index was out of bounds");
		return false;
	}

	rc = filePath_.empty() ? device_.open(deviceArray[which].getUri()) : device_.open(filePath_.c_str());
	if (rc != Status::STATUS_OK)
	{
		logger_->error("Could not open device");
		return false;
	}

	if (device_.isFile())
	{
		rc = device_.setProperty<int>(openni::DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED, 1);
		LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not set repeat mode enabled for .oni file playback");
	}

	rc = device_.setDepthColorSyncEnabled(true);
	LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not framelock depth and color streams");

	rc = depthStream_.create(device_, openni::SENSOR_DEPTH);
	LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not create depth stream");

	rc = colorStream_.create(device_, openni::SENSOR_COLOR);
	LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not create color stream");

	VideoMode colorstreamMode;
	colorstreamMode.setPixelFormat(PixelFormat::PIXEL_FORMAT_RGB888);
	colorstreamMode.setResolution(rgbWidth_, rgbHeight_);
	colorstreamMode.setFps(rgbFPS_);

	VideoMode depthstreamMode;
	depthstreamMode.setPixelFormat(PixelFormat::PIXEL_FORMAT_DEPTH_1_MM);
	depthstreamMode.setResolution(zWidth_,zHeight_);
	depthstreamMode.setFps(zFPS_);

	if (!device_.isFile())
	{
		rc = colorStream_.setVideoMode(colorstreamMode);
		LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not set color stream mode");

		rc = depthStream_.setVideoMode(depthstreamMode);
		LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not set depth stream mode");
	}

	if (colorStream_.start() != Status::STATUS_OK)
	{
		logger_->error("Could not start either color stream");
		return false;
	}

	if (depthStream_.start() != Status::STATUS_OK)
	{
		logger_->error("Could not start either depth stream");
		return false;
	}

	rc = device_.setImageRegistrationMode(openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	LOG4CXX_ASSERT(logger_, rc == Status::STATUS_OK, "Could not register the depth map to color map");

	colorstreamMode = colorStream_.getVideoMode();
	depthstreamMode = depthStream_.getVideoMode();

	rgbImage_ = cv::Mat(colorstreamMode.getResolutionY(), colorstreamMode.getResolutionX(), CV_8UC3);
	rgbaImage_ = cv::Mat(colorstreamMode.getResolutionY(), colorstreamMode.getResolutionX(), CV_8UC4);
	depthImage_ = cv::Mat(depthstreamMode.getResolutionY(), depthstreamMode.getResolutionX(), CV_16UC1);

	rgbWidth_ = colorstreamMode.getResolutionX();
	rgbHeight_ = colorstreamMode.getResolutionY();
	rgbFPS_ = (float)colorstreamMode.getFps();
	zWidth_ = depthstreamMode.getResolutionX();
	zHeight_ = depthstreamMode.getResolutionY();
	zFPS_ = depthstreamMode.getFps();

	if (depthStream_.getProperty<float>(ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &hov_) != Status::STATUS_OK)
	{
		logger_->error("Could not get horizontal field-of-view from depth stream");
		return false;
	}

	if (depthStream_.getProperty<float>(ONI_STREAM_PROPERTY_VERTICAL_FOV, &vov_) != Status::STATUS_OK)
	{
		logger_->error("Could not get vertical field-of-view from depth stream");
		return false;
	}

	worldConvertCache_.xzFactor = tan(hov_ / 2) * 2;
	worldConvertCache_.yzFactor = tan(vov_ / 2) * 2;
	worldConvertCache_.resolutionX = zWidth_;
	worldConvertCache_.resolutionY = zHeight_;
	worldConvertCache_.halfResX = worldConvertCache_.resolutionX / 2;
	worldConvertCache_.halfResY = worldConvertCache_.resolutionY / 2;
	worldConvertCache_.coeffX = worldConvertCache_.resolutionX / worldConvertCache_.xzFactor;
	worldConvertCache_.coeffY = worldConvertCache_.resolutionY / worldConvertCache_.yzFactor;

	pointCloud_ = createNewCloud();

	isOpen_ = true;

	return isOpen_;
}


void HoloCaptureOpenNI2::waitAndGetNextFrame(cv::Mat& rgbaImage, cv::Mat& zImage)
{
	if (isOpen())
	{
		openni::VideoFrameRef color;
		openni::VideoFrameRef depth;

		colorStream_.readFrame(&color);
		depthStream_.readFrame(&depth);

		rgbImage_ = cv::Mat(cv::Size(rgbWidth_, rgbHeight_), CV_8UC3, (void*)color.getData(), color.getStrideInBytes() );
		cv::cvtColor(rgbImage_, rgbaImage_, CV_BGR2RGBA, 4);

		memcpy(depthImage_.datastart, depth.getData(), depth.getDataSize());

		rgbaImage = rgbaImage_;
		zImage = depthImage_;
	}
	
}

void HoloCaptureOpenNI2::waitAndGetNextPointCloud(HoloCloudPtr& pointCloud)
{
	if (isOpen_)
	{
		auto futureCloud = std::async(std::launch::async, &HoloCaptureOpenNI2::createNewCloud, this);

		openni::VideoFrameRef color;
		openni::VideoFrameRef depth;

		colorStream_.readFrame(&color);
		depthStream_.readFrame(&depth);

		const unsigned char * pp = (const unsigned char*)color.getData();
		const unsigned short *depthPix = (unsigned short*)depth.getData();

		HoloPoint3D * point = &pointCloud_->points[0];

		float depthVal = HOLO_CLOUD_BAD_POINT;

		for (int i = 0, idx = 0; i < zHeight_; i++)
		{
			for (int j = 0; j < zWidth_; j++, idx++, pp+=3, depthPix++, point++)
			{
				if (*depthPix <= 0 || *depthPix > 1000)
				{
					point->x = point->y = point->z = HOLO_CLOUD_BAD_POINT;
					point->r = point->g = point->b = 0;
					point->a = 255;
					continue;
				}

				//openni::CoordinateConverter::convertDepthToWorld(depthStream_, i, j, *depthPix, &x, &y, &z);
				depthVal = static_cast<float>(*depthPix) * 0.001f;

				point->x = (.5f - static_cast<float>(j) / worldConvertCache_.resolutionY) * depthVal * worldConvertCache_.yzFactor;
				point->y = -(static_cast<float>(i) / worldConvertCache_.resolutionX - .5f) * depthVal * worldConvertCache_.xzFactor;
				point->z = depthVal;

				point->r = *pp;
				point->g = *(pp + 1);
				point->b = *(pp + 2);
				point->a = 0;
			}
		}

		pointCloud = (HoloCloudPtr)futureCloud.get();

		pointCloud_.swap(pointCloud);
	}
}

HoloCloudPtr HoloCaptureOpenNI2::createNewCloud()
{
	auto cloud = HoloCloudPtr(new HoloCloud(zWidth_, zHeight_));
	cloud->is_dense = false;
	cloud->sensor_origin_.setZero();
	cloud->sensor_orientation_.setIdentity();
	return cloud;
}

void HoloCaptureOpenNI2::deinit()
{
	if (isOpen())
	{
		depthStream_.stop();
		colorStream_.stop();
		deinitOpenNI2();
		isOpen_ = false;
		LOG4CXX_INFO(logger_, "Deinitialized OpenNI2 device");
	}	
}

std::list<std::string> HoloCaptureOpenNI2::enumerateDevices()
{
	LOG4CXX_INFO(logger_, "Enumerating OpenNI2 devices...")
	std::list<std::string> deviceList;
	bool wasOpenNI2AlreadyInit = isOpenNI2Init_ ? true : false;

	initOpenNI2();

	openni::Array<openni::DeviceInfo> deviceArray;

	OpenNI::enumerateDevices(&deviceArray);

	for (int i = 0; i < deviceArray.getSize(); i++)
	{
		deviceList.push_back(std::string(deviceArray[i].getName()));
		LOG4CXX_DEBUG(logger_, "Found device " << deviceArray[0].getName() << " with URI " << deviceArray[0].getUri());
	}

	LOG4CXX_INFO(logger_, "Done enumerating OpenNI devices")

	if (!wasOpenNI2AlreadyInit)
		deinitOpenNI2();

	return deviceList;
}

bool HoloCaptureOpenNI2::initOpenNI2()
{
	Status rc = openni::STATUS_OK;
	if (!isOpenNI2Init_)
	{
		LOG4CXX_DEBUG(logger_, "Calling OpenNI::initialize...");
		rc = OpenNI::initialize();
		if (rc != Status::STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "OpenNI2 library could not initialize. Error status: " << rc)
			throw;
		}
		else
		{
			isOpenNI2Init_ = true;
			LOG4CXX_INFO(logger_, "Initialized OpenNI2 library")
		}
	}

	return isOpenNI2Init_;
}

void HoloCaptureOpenNI2::deinitOpenNI2()
{
	LOG4CXX_DEBUG(logger_,"Calling OpenNI2::shutdown...");
	openni::OpenNI::shutdown();
	isOpenNI2Init_ = false;
	LOG4CXX_INFO(logger_, "Deinitialized OpenNI2 library");

}