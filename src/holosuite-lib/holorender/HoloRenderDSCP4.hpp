#pragma once
#ifdef ENABLE_HOLO_DSCP4

#include "IHoloRender.hpp"
#include "HoloRenderObjectTracker.hpp"

#include <log4cxx/log4cxx.h>

#include <dscp4/dscp4.h>

namespace holo
{
	namespace render
	{
		class HoloRenderDSCP4 : public IHoloRender
		{
		public:
			HoloRenderDSCP4();
			HoloRenderDSCP4(render_options_t *render_options,
				algorithm_options_t *algorithm_options,
				display_options_t display_options, unsigned int voxelSize,
				unsigned int verbosity, void * logAppender = nullptr);
			~HoloRenderDSCP4();

			bool init();
			void deinit();

			void updateRemotePointCloud(HoloCloudPtr && pointCloud);
			void updateLocalPointCloud(HoloCloudPtr && pointCloud);
			void* getContext() { return nullptr; }

			void addObjectTracker(boost::shared_ptr<HoloRenderObjectTracker> & objectTracker)
			{
				objectTracker_ = objectTracker;
			}

			void removeObjectTracker()
			{
				objectTracker_.reset();
			}

		private:

			dscp4_context_t context_;
			
			HoloCloudPtr localCloud_;
			HoloCloudPtr remoteCloud_;

			render_options_t renderOptions_;
			algorithm_options_t algorithmOptions_;
			display_options_t displayOptions_;

			boost::shared_ptr<HoloRenderObjectTracker> objectTracker_;

			unsigned int voxelSize_;

			bool firstRun_;

		};
	}
}

#endif