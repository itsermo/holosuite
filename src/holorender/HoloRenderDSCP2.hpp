#pragma once
#include "IHoloRender.hpp"

namespace holo
{
	namespace render
	{
		class HoloRenderDSCP2 : IHoloRender
		{
		public:
			HoloRenderDSCP2();
			~HoloRenderDSCP2();

			bool init();
			void deinit();
			//virtual void updateFromMats(cv::Mat rgbaImage, cv::Mat depthImage) = 0;
			void updateFromPointCloud(HoloCloudPtr && pointCloud);
			void* getContext() { return nullptr; }

		private:

		};
	}
}