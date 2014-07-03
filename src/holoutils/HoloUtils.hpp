#pragma once
#include "../common/CommonDefs.hpp"
namespace holo
{
	namespace utils
	{

		//converts rgba + z pictures into a point cloud and reprojects to real-world coordinates quickly
		void ReprojectToRealWorld(HoloCloudPtr& cloudOut, HoloRGBAZMat& rgbaz, holo::capture::WorldConvertCache& worldConvertCache);

		//converts rgb888 to rgba8888, (aka rgb to rgba)
		void ConvertRGBToRGBA(cv::Mat& rgbMat, cv::Mat& rgbaMatOut);
	}
}