#pragma once
#include "IHoloRender.hpp"

#include "HoloRenderVisualizer.hpp"

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

		private:

		};

		//HoloRenderGenerator::HoloRenderGenerator()
		//{
		//}

		//HoloRenderGenerator::~HoloRenderGenerator()
		//{
		//}
	}
}