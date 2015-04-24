#pragma once
#include "IHoloRender.hpp"
#include "HoloRenderVisualizer.hpp"

#ifdef ENABLE_HOLO_AUDIO
#include "HoloRenderAudioPortaudio.hpp"
#endif

#ifdef ENABLE_HOLO_DSCP2
#include "HoloRenderDSCP2.hpp"
#endif

#ifdef ENABLE_HOLO_DSCP4
#include "HoloRenderDSCP4.hpp"
#endif

#include "HoloRenderOpenGL.hpp"

#include <memory>

namespace holo
{
	namespace render
	{
		class HoloRenderGenerator
		{
		public:
			HoloRenderGenerator();
			~HoloRenderGenerator();

			static boost::shared_ptr<IHoloRender> fromPCLVisualizer()
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderVisualizer());
			}

			static boost::shared_ptr<IHoloRender> fromPCLVisualizer(int voxelSize, bool enableMeshConstruction)
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderVisualizer(voxelSize, enableMeshConstruction));
			}

#ifdef ENABLE_HOLO_AUDIO
			static std::unique_ptr<IHoloRenderAudio> fromPortaudio()
			{
				return std::unique_ptr<IHoloRenderAudio>(new HoloRenderAudioPortaudio());
			}

			static std::unique_ptr<IHoloRenderAudio> fromPortaudio(HoloAudioFormat audioFormat)
			{
				return std::unique_ptr<IHoloRenderAudio>(new HoloRenderAudioPortaudio(audioFormat));
			}
#endif

#ifdef ENABLE_HOLO_DSCP2
			static boost::shared_ptr<IHoloRender> fromDSCP2()
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderDSCP2());
			}

			static boost::shared_ptr<IHoloRender> fromDSCP2(int headNumber, std::string displayEnv)
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderDSCP2(headNumber, displayEnv));
			}
#endif

#ifdef ENABLE_HOLO_DSCP4
			static boost::shared_ptr<IHoloRender> fromDSCP4()
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderDSCP4());
			}

			static boost::shared_ptr<IHoloRender> fromDSCP4(render_options_t *render_options,
				algorithm_options_t *algorithm_options,
				display_options_t display_options,
				unsigned int verbosity, void * logAppender = nullptr)
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderDSCP4(render_options, algorithm_options, display_options, verbosity, logAppender));
			}
#endif

			static boost::shared_ptr<IHoloRender> fromOpenGL()
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderOpenGL());
			}

			static boost::shared_ptr<IHoloRender> fromOpenGL(int voxelSize, bool enableZSpaceRendering)
			{
				return boost::shared_ptr<IHoloRender>(new HoloRenderOpenGL(voxelSize, enableZSpaceRendering));
			}

		};

		//HoloRenderGenerator::HoloRenderGenerator()
		//{
		//}

		//HoloRenderGenerator::~HoloRenderGenerator()
		//{
		//}
	}
}