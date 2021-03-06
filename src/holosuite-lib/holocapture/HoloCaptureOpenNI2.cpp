#include "HoloCaptureOpenNI2.hpp"
#include "../holoutils/HoloUtils.hpp"
#include <future>

#ifdef TRACE_LOG_ENABLED
#include <chrono>
#include <ctime>
#endif

using namespace holo;
using namespace holo::capture;
using namespace openni;

HoloCaptureOpenNI2::HoloCaptureOpenNI2() : IHoloCapture(),
rgbWidth_(HOLO_CAPTURE_DEFAULT_RGB_WIDTH),
rgbHeight_(HOLO_CAPTURE_DEFAULT_RGB_HEIGHT),
rgbFPS_(HOLO_CAPTURE_DEFAULT_RGB_FPS),
zWidth_(HOLO_CAPTURE_DEFAULT_Z_WIDTH),
zHeight_(HOLO_CAPTURE_DEFAULT_Z_HEIGHT),
zFPS_(HOLO_CAPTURE_DEFAULT_Z_FPS),
isOpen_(false),
isOpenNI2Init_(false), worldConvertCache_()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.openni2");

	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object instantiated");
}

HoloCaptureOpenNI2::HoloCaptureOpenNI2(std::string filePath) :
	IHoloCapture(),
	rgbWidth_(HOLO_CAPTURE_DEFAULT_RGB_WIDTH),
	rgbHeight_(HOLO_CAPTURE_DEFAULT_RGB_HEIGHT),
	rgbFPS_(HOLO_CAPTURE_DEFAULT_RGB_FPS),
	zWidth_(HOLO_CAPTURE_DEFAULT_Z_WIDTH),
	zHeight_(HOLO_CAPTURE_DEFAULT_Z_HEIGHT),
	zFPS_(HOLO_CAPTURE_DEFAULT_Z_FPS),
	isOpen_(false),
	isOpenNI2Init_(false),
	inputFilePath_(filePath),
	worldConvertCache_(),
	shouldRecord_(false)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.openni2");

	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object instantiated");
}

HoloCaptureOpenNI2::HoloCaptureOpenNI2(unsigned int rgbWidth,
	unsigned int rgbHeight,
	float rgbFPS,
	unsigned int zWidth,
	unsigned int zHeight,
	float zFPS, bool shouldRecord,
	std::string fileName) :
	rgbWidth_(rgbWidth),
	rgbHeight_(rgbHeight),
	rgbFPS_(rgbFPS),
	zWidth_(zWidth),
	zHeight_(zHeight),
	zFPS_(zFPS),
	isOpen_(false),
	isOpenNI2Init_(false),
	outputFilePath_(fileName),
	worldConvertCache_(),
	shouldRecord_(shouldRecord)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.openni2");

	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object instantiated");
}

HoloCaptureOpenNI2::~HoloCaptureOpenNI2()
{
	deinit();
	LOG4CXX_DEBUG(logger_, "HoloCaptureOpenNI2 object destroyed");
}

bool HoloCaptureOpenNI2::init(int which)
{
	LOG4CXX_INFO(logger_, "Initializing OpenNI2 capture device...")
	LOG4CXX_INFO(logger_, "Camera index: " << which);
	LOG4CXX_INFO(logger_, "RGB Width: " << rgbWidth_);
	LOG4CXX_INFO(logger_, "RGB Height: " << rgbHeight_);
	LOG4CXX_INFO(logger_, "RGB Framerate: " << rgbFPS_);
	LOG4CXX_INFO(logger_, "Z Width: " << zWidth_);
	LOG4CXX_INFO(logger_, "Z Height: " << zHeight_);
	LOG4CXX_INFO(logger_, "Z FPS: " << zFPS_);

	Status rc = STATUS_OK;

	initOpenNI2();

	openni::Array<openni::DeviceInfo> deviceArray;
	OpenNI::enumerateDevices(&deviceArray);

	if (deviceArray.getSize() == 0 && inputFilePath_.empty())
	{
		LOG4CXX_ERROR(logger_, "Did not find any OpenNI2 devices");
		return false;
	}
	else if ((deviceArray.getSize() < which || which < 0) && inputFilePath_.empty())
	{
		LOG4CXX_ERROR(logger_, "Depth camera device selection index was out of bounds");
		return false;
	}
	
	if (inputFilePath_.empty())
	{
		LOG4CXX_INFO(logger_, "Opening device " << deviceArray[which].getVendor() << " " << deviceArray[which].getName() << " @ " << deviceArray[which].getUri());
	}
	else
		LOG4CXX_INFO(logger_, "Opening ONI file \"" << inputFilePath_ << "\"...");

	rc = inputFilePath_.empty() ? device_.open(deviceArray[which].getUri()) : device_.open(inputFilePath_.c_str());
	if (rc != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not open device");
		return false;
	}

	if (device_.isFile())
	{
		rc = device_.setProperty<int>(openni::DEVICE_PROPERTY_PLAYBACK_REPEAT_ENABLED, 1);
		if (rc != STATUS_OK)
		{
			LOG4CXX_WARN(logger_, "Could not set repeat mode enabled for .oni file playback");
		}
	}

	rc = device_.setDepthColorSyncEnabled(true);
	if (rc != STATUS_OK)
		LOG4CXX_WARN(logger_, "Could not framelock depth and color streams");

	rc = depthStream_.create(device_, openni::SENSOR_DEPTH);
	if (rc != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not create depth stream");
		return false;
	}

	rc = colorStream_.create(device_, openni::SENSOR_COLOR);
	if (rc != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not create color stream");
		return false;
	}

	VideoMode colorstreamMode;
	colorstreamMode.setPixelFormat(PIXEL_FORMAT_RGB888);
	colorstreamMode.setResolution(rgbWidth_, rgbHeight_);
	colorstreamMode.setFps(rgbFPS_);

	VideoMode depthstreamMode;
	depthstreamMode.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);
	depthstreamMode.setResolution(zWidth_,zHeight_);
	depthstreamMode.setFps(zFPS_);

	if (!device_.isFile())
	{
		rc = colorStream_.setVideoMode(colorstreamMode);
		if (rc != STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "Could not set color stream mode");
			return false;
		}

		rc = depthStream_.setVideoMode(depthstreamMode);
		if (rc != STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "Could not set depth stream mode");
			return false;
		}
	}

	if (colorStream_.start() != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not start either color stream");
		return false;
	}

	//if (colorStream_.addNewFrameListener(&colorListener_) != STATUS_OK)
	//{
	//	LOG4CXX_ERROR(logger_, "Could not set color stream event listener");
	//	return false;
	//}

	if (depthStream_.start() != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not start either depth stream");
		return false;
	}

	//if (depthStream_.addNewFrameListener(&depthListener_) != STATUS_OK)
	//{
	//	LOG4CXX_ERROR(logger_, "Could not set depth stream event listener");
	//	return false;
	//}

	rc = device_.setImageRegistrationMode(openni::ImageRegistrationMode::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
	if (rc != STATUS_OK)
	{
		LOG4CXX_WARN(logger_, "Could not register the depth map to color map");
	}

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

	if (depthStream_.getProperty<float>(ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &zHOV_) != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not get horizontal field-of-view from depth stream");
		return false;
	}

	if (!(zHOV_ > 0))
	{
		zHOV_ = 58.f * M_PI / 180.f;
		LOG4CXX_WARN(logger_, "Got a bogus horizontal field-of-view from depth stream. Setting HOV to 58 deg.")
	}

	if (depthStream_.getProperty<float>(ONI_STREAM_PROPERTY_VERTICAL_FOV, &zVOV_) != STATUS_OK)
	{
		LOG4CXX_ERROR(logger_, "Could not get vertical field-of-view from depth stream");
		return false;
	}

	if (!(zVOV_ > 0))
	{
		zVOV_ = 45.f * M_PI / 180.f;
		LOG4CXX_WARN(logger_, "Got a bogus vertical field-of-view from depth stream. Setting VOV to 45 deg.")
	}

	if (colorStream_.getProperty<float>(ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &rgbHOV_) != STATUS_OK)
	{
		LOG4CXX_WARN(logger_, "Could not get HOV from color stream. Using depth value instead.");
		rgbHOV_ = zHOV_;
	}

	if (!(rgbHOV_ > 0))
	{
		rgbHOV_ = 58.f * M_PI / 180.f;
		LOG4CXX_WARN(logger_, "Got a bogus horizontal field-of-view from color stream. Setting HOV to 58 deg.")
	}

	if (colorStream_.getProperty<float>(ONI_STREAM_PROPERTY_VERTICAL_FOV, &rgbVOV_) != STATUS_OK)
	{
		LOG4CXX_WARN(logger_, "Could not get VOV from color stream. Using depth value instead.");
		rgbVOV_ = zVOV_;
	}

	if (!(rgbVOV_ > 0))
	{
		rgbVOV_ = 45.f * M_PI / 180.f;
		LOG4CXX_WARN(logger_, "Got a bogus vertical field-of-view from color stream. Setting VOV to 58 deg.")
	}

	// constants for reprojecting the point cloud to real-world coordinates
	worldConvertCache_.xzFactor = tan(rgbHOV_ / 2) * 2;
	worldConvertCache_.yzFactor = tan(rgbVOV_ / 2) * 2;
	worldConvertCache_.resolutionX = zWidth_;
	worldConvertCache_.resolutionY = zHeight_;
	worldConvertCache_.halfResX = worldConvertCache_.resolutionX / 2;
	worldConvertCache_.halfResY = worldConvertCache_.resolutionY / 2;
	worldConvertCache_.coeffX = worldConvertCache_.resolutionX / worldConvertCache_.xzFactor;
	worldConvertCache_.coeffY = worldConvertCache_.resolutionY / worldConvertCache_.yzFactor;

	pointCloud_ = createNewCloud();

	isOpen_ = true;

	LOG4CXX_INFO(logger_, "OpenNI2 capture device intitalized")

		//chdir("../");
	if (shouldRecord_)
	{
		LOG4CXX_INFO(logger_, "Starting OpenNI2 recorder with file path \"" << outputFilePath_ << "\" ...")
		if (recorder_.create(outputFilePath_.c_str()) != STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "Could not create OpenNI2 recorder device");
			return false;
		}

		if (recorder_.attach(colorStream_) != STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "Could not attach color stream to OpenNI2 recorder device");
			return false;
		}
		
		if (recorder_.attach(depthStream_, false))
		{
			LOG4CXX_ERROR(logger_, "Could not attach depth stream to OpenNI2 recorder device");
			return false;
		}

		if (recorder_.start() != STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "Could not start OpenNI2 recorder device");
			return false;
		}
		LOG4CXX_INFO(logger_, "OpenNI2 recorder device started");
	}

	return isOpen_;
}


void HoloCaptureOpenNI2::waitAndGetNextFrame(cv::Mat& rgbaImage, cv::Mat& zImage)
{
	if (isOpen_)
	{
		openni::VideoFrameRef color;
		openni::VideoFrameRef depth;

		colorStream_.readFrame(&color);
		depthStream_.readFrame(&depth);
#ifdef TRACE_LOG_ENABLED
		auto startTime = std::chrono::system_clock::now();
#endif
		//auto depthFuture = std::async(std::launch::async, &holo::capture::HoloCaptureOpenNI2Listener::getDepthFrame, &depthListener_, std::ref(zImage));
		
		//colorListener_.getColorFrame(rgbaImage);
		
		//processColorFrame();
		
		//depthFuture.get();

		rgbaImage = rgbaImage_;
		zImage = depthImage_;

		rgbImage_ = cv::Mat(cv::Size(rgbWidth_, rgbHeight_), CV_8UC3, (void*)color.getData(), color.getStrideInBytes() );
		
		auto futureRGBA = std::async(std::launch::async, &holo::utils::ConvertRGBToRGBA, std::ref(rgbImage_), std::ref(rgbaImage_));
		//cv::cvtColor(rgbImage_, rgbaImage_, CV_BGR2RGBA, 4);

		//memcpy(depthImage_.datastart, depth.getData(), depth.getDataSize());
		
		short * dsrc = (short*)depth.getData();
		short * ddest = (short*)depthImage_.data;
		for (int i = 0; i < zWidth_ * zHeight_; i++, dsrc++, ddest++)
			*ddest = *dsrc < 1800 ? *dsrc : 0;

		zImage = depthImage_;


		futureRGBA.get();


		rgbaImage = rgbaImage_;

#ifdef TRACE_LOG_ENABLED
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
		LOG4CXX_TRACE(logger_, "OpenNI2 frame copied Z-frame and converted RGB to RGBA in " << duration.count() << "ms");
#endif
	}
	
}

void HoloCaptureOpenNI2::waitAndGetNextPointCloud(HoloCloudPtr& pointCloud)
{
	if (isOpen_)
	{
		auto futureCloud = std::async(std::launch::async, &HoloCaptureOpenNI2::createNewCloud, this);

		HoloRGBAZMat mats;

		waitAndGetNextFrame(mats.rgba, mats.z);

#ifdef TRACE_LOG_ENABLED
		auto startTime = std::chrono::system_clock::now();
#endif
		holo::utils::ReprojectToRealWorld(pointCloud_, mats, worldConvertCache_);

		pointCloud = (HoloCloudPtr)futureCloud.get();

		pointCloud_.swap(pointCloud);

#ifdef TRACE_LOG_ENABLED
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - startTime);
		LOG4CXX_TRACE(logger_, "OpenNI2 frame copied Z-frame and converted RGB to RGBA in " << duration.count() << "ms");
#endif

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
	if (isOpen_)
	{

		if (shouldRecord_)
		{
			recorder_.stop();
			recorder_.destroy();
		}
		//depthStream_.removeNewFrameListener(&depthListener_);
		//colorStream_.removeNewFrameListener(&colorListener_);

		depthStream_.stop();
		colorStream_.stop();

		deinitOpenNI2();
		isOpen_ = false;
		LOG4CXX_INFO(logger_, "Deinitialized OpenNI2 device");
	}	
}

std::vector<std::string> HoloCaptureOpenNI2::enumerateDevices()
{
	LOG4CXX_DEBUG(logger_, "Enumerating OpenNI2 devices...")
	std::vector<std::string> deviceList;
	bool wasOpenNI2AlreadyInit = isOpenNI2Init_ ? true : false;

	initOpenNI2();

	openni::Array<openni::DeviceInfo> deviceArray;

	OpenNI::enumerateDevices(&deviceArray);

	for (int i = 0; i < deviceArray.getSize(); i++)
	{
		deviceList.push_back(std::string(deviceArray[i].getName()));
		LOG4CXX_DEBUG(logger_, "Found device " << deviceArray[0].getName() << " @ " << deviceArray[0].getUri());
	}

	LOG4CXX_DEBUG(logger_, "Done enumerating OpenNI devices")

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
		if (rc != STATUS_OK)
		{
			LOG4CXX_ERROR(logger_, "OpenNI2 library could not initialize. Error status: " << rc)
			return false;
		}
		else
		{
			isOpenNI2Init_ = true;
			LOG4CXX_DEBUG(logger_, "Initialized OpenNI2 library")
		}
	}

	return isOpenNI2Init_;
}

void HoloCaptureOpenNI2::deinitOpenNI2()
{
	LOG4CXX_DEBUG(logger_,"Calling OpenNI2::shutdown...");
	openni::OpenNI::shutdown();
	isOpenNI2Init_ = false;
	LOG4CXX_DEBUG(logger_, "Deinitialized OpenNI2 library");

}

