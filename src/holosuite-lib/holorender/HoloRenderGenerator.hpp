#pragma once
#include "IHoloRender.hpp"
#include "HoloRenderVisualizer.hpp"

#ifdef ENABLE_HOLO_AUDIO
#include "HoloRenderAudioPortaudio.hpp"
#endif

#ifdef ENABLE_HOLO_DSCP2
#include "HoloRenderDSCP2.hpp"
#endif

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

			static std::unique_ptr<IHoloRender> fromPCLVisualizer()
			{
				return std::unique_ptr<IHoloRender>(new HoloRenderVisualizer());
			}

			static std::unique_ptr<IHoloRender> fromPCLVisualizer(int voxelSize, bool enableMeshConstruction)
			{
				return std::unique_ptr<IHoloRender>(new HoloRenderVisualizer(voxelSize, enableMeshConstruction));
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
			static std::unique_ptr<IHoloRender> fromDSCP2()
			{
				return std::unique_ptr<IHoloRender>(new HoloRenderDSCP2());
			}

			static std::unique_ptr<IHoloRender> fromDSCP2(int headNumber, std::string displayEnv)
			{
				return std::unique_ptr<IHoloRender>(new HoloRenderDSCP2(headNumber, displayEnv));
			}
#endif

		};

		//HoloRenderGenerator::HoloRenderGenerator()
		//{
		//}

		//HoloRenderGenerator::~HoloRenderGenerator()
		//{
		//}
	}
}