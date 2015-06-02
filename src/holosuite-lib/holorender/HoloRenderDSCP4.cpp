#ifdef ENABLE_HOLO_DSCP4

#include "HoloRenderDSCP4.hpp"
#include <dscp4/dscp4_defs.h>

using namespace holo;
using namespace holo::render;

HoloRenderDSCP4::HoloRenderDSCP4() : IHoloRender(),
	voxelSize_(3),
	firstRun_(true)
{
	context_ = dscp4_CreateContextDefault();
}

HoloRenderDSCP4::HoloRenderDSCP4(render_options_t *render_options,
	algorithm_options_t *algorithm_options,
	display_options_t display_options, unsigned int voxelSize,
	unsigned int verbosity, void * logAppender) : IHoloRender(),
	voxelSize_(voxelSize),
	firstRun_(true)
{
	context_ = dscp4_CreateContext(render_options, algorithm_options, display_options, verbosity, logAppender);
}

HoloRenderDSCP4::~HoloRenderDSCP4()
{
	dscp4_DestroyContext(&context_);
}

bool HoloRenderDSCP4::init(bool enableMirrorVisualFeedback)
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
	if (firstRun_)
	{
		dscp4_AddPointCloud(context_, "remoteCloud", remoteCloud_->points.size(), voxelSize_, remoteCloud_->points.data());
		dscp4_ScaleObject(context_, "remoteCloud", 3.7f, 3.7f, -3.7f);
		dscp4_TranslateObject(context_, "remoteCloud", -0.05f, 0.f, -0.75f);
		firstRun_ = false;
	}
	else
	{
		dscp4_UpdatePointCloud(context_, "remoteCloud", voxelSize_, remoteCloud_->points.data());
	}
}

void HoloRenderDSCP4::updateLocalPointCloud(HoloCloudPtr && pointCloud)
{
	localCloud_ = pointCloud;
}

#endif
