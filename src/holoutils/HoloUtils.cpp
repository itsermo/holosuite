#include "HoloUtils.hpp"

using namespace holo;
using namespace holo::utils;

void holo::utils::ReprojectToRealWorld(HoloCloudPtr& cloudOut, HoloRGBAZMat& rgbaz, holo::capture::WorldConvertCache& worldConvertCache)
{
	const unsigned int * pp = (unsigned int*)rgbaz.rgba.datastart;
	const unsigned short *depthPix = (unsigned short*)rgbaz.z.datastart;

	HoloPoint3D * point = &cloudOut->points[0];

	float depthVal = HOLO_CLOUD_BAD_POINT;

	for (int i = 0, idx = 0; i < rgbaz.z.rows; i++)
	{
		for (int j = 0; j < rgbaz.z.cols; j++, idx++, pp ++, depthPix++, point++)
		{
			if (*depthPix <= 0 || *depthPix > 1000)
			{
				point->x = point->y = point->z = HOLO_CLOUD_BAD_POINT;
				point->r = point->g = point->b = 0;
				point->a = 255;
				continue;
			}

			depthVal = static_cast<float>(*depthPix) * 0.001f;

			point->x = (.5f - static_cast<float>(j) / worldConvertCache.resolutionY) * depthVal * worldConvertCache.yzFactor;
			point->y = -(static_cast<float>(i) / worldConvertCache.resolutionX - .5f) * depthVal * worldConvertCache.xzFactor;
			point->z = depthVal;

			point->rgba = *pp;
		}
	}
}
