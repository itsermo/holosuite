#pragma once
#include "IHoloRender.hpp"
#include "HoloRenderVisualizer.hpp"

#ifdef ENABLE_HOLO_AUDIO
#include "HoloRenderAudioPortaudio.hpp"
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

			static std::unique_ptr<IHoloRender> fromPCLVisualizer(int voxelSize, int zWidth, int zHeight)
			{
				return std::unique_ptr<IHoloRender>(new HoloRenderVisualizer(voxelSize, zWidth, zHeight));
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

		};

		//HoloRenderGenerator::HoloRenderGenerator()
		//{
		//}

		//HoloRenderGenerator::~HoloRenderGenerator()
		//{
		//}
	}
}