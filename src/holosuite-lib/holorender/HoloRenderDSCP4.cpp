#ifdef ENABLE_HOLO_DSCP4

#include "HoloRenderDSCP4.hpp"
#include <dscp4_defs.h>

using namespace holo;
using namespace holo::render;

HoloRenderDSCP4::HoloRenderDSCP4() : IHoloRender()
{
	context_ = dscp4_CreateContextDefault();
}

HoloRenderDSCP4::HoloRenderDSCP4(render_options_t *render_options,
	algorithm_options_t *algorithm_options,
	display_options_t display_options,
	unsigned int verbosity, void * logAppender) : IHoloRender()
{
	context_ = dscp4_CreateContext(render_options, algorithm_options, display_options, verbosity, logAppender);
}

HoloRenderDSCP4::~HoloRenderDSCP4()
{
	dscp4_DestroyContext(&context_);
}

bool HoloRenderDSCP4::init()
{
	if (context_)
		return dscp4_InitRenderer(context_);
	else
		return false;
}

void HoloRenderDSCP4::deinit()
{
	if (context_)
		dscp4_DeinitRenderer(context_);
}

void HoloRenderDSCP4::updateRemotePointCloud(HoloCloudPtr && pointCloud)
{
	remoteCloud_ = pointCloud;
	dscp4_AddPointCloud(context_, "remoteCloud", remoteCloud_->points.size(), remoteCloud_->points.data());
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void HoloRenderDSCP4::updateLocalPointCloud(HoloCloudPtr && pointCloud)
{
	localCloud_ = pointCloud;
}

#endif
