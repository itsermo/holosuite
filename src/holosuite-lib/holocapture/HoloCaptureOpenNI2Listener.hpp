#pragma once
#include "../holocommon/CommonDefs.hpp"

#include <log4cxx/log4cxx.h>
#include <opencv2/opencv.hpp>
#include <OpenNI.h>
#include <mutex>
#include <condition_variable>

#define HOLO_CAPTURE_OPENNI2_MUTEX_TIMEOUT_MS 30

namespace holo
{
	namespace capture
	{
		class HoloCaptureOpenNI2Listener : public openni::VideoStream::NewFrameListener
		{
		public:
			HoloCaptureOpenNI2Listener();
			~HoloCaptureOpenNI2Listener();

			void onNewFrame(openni::VideoStream& stream);

			void getDepthFrame(cv::Mat& depthOut);
			void getColorFrame(cv::Mat& colorOut);

		private:

			openni::VideoFrameRef colorFrame_;
			openni::VideoFrameRef depthFrame_;

			bool haveNewColorFrame_;
			bool haveNewDepthFrame_;

			std::mutex colorFrameMutex_;
			std::mutex depthFrameMutex_;
			std::condition_variable colorFrameCV_;
			std::condition_variable depthFrameCV_;

			log4cxx::LoggerPtr logger_;

		};
	}
}
