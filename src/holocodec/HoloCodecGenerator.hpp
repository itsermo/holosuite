#pragma once
#include "IHoloCodec.hpp"
#include "HoloCodecPassthroughCloud.hpp"
#include "HoloCodecOctree.hpp"
#include "HoloCodecH264.hpp"

#include <memory>

namespace holo
{
	namespace codec
	{
		class HoloCodecGenerator
		{
		public:
			HoloCodecGenerator();
			~HoloCodecGenerator();

			static std::unique_ptr<IHoloCodec<holo::HoloCloud>> fromPCLPassthrough()
			{
				return std::unique_ptr<IHoloCodec<holo::HoloCloud>>(new HoloCodecPassthroughCloud());
			}

			static std::unique_ptr<IHoloCodec<holo::HoloCloud>> fromPCLOctreeCompression()
			{
				return std::unique_ptr<IHoloCodec<holo::HoloCloud>>(new HoloCodecOctree());
			}

			static std::unique_ptr<IHoloCodec<holo::HoloCloud>> fromPCLOctreeCompression(HoloCodecOctreeEncodeArgs args)
			{
				return std::unique_ptr<IHoloCodec<holo::HoloCloud>>(new HoloCodecOctree(args));
			}

			static std::unique_ptr<IHoloCodec<holo::HoloRGBAZMat>> fromH264()
			{
				return std::unique_ptr<IHoloCodec<holo::HoloRGBAZMat>>(new HoloCodecH264());
			}

			static std::unique_ptr<IHoloCodec<holo::HoloRGBAZMat>> fromH264(int bitRate, int width, int height, AVRational timeBase, int gopSize, int maxBFrames, AVPixelFormat pixFmt)
			{
				return std::unique_ptr<IHoloCodec<holo::HoloRGBAZMat>>(new HoloCodecH264(bitRate, width, height, timeBase, gopSize, maxBFrames, pixFmt));
			}

		private:

		};

		HoloCodecGenerator::HoloCodecGenerator()
		{
		}

		HoloCodecGenerator::~HoloCodecGenerator()
		{
		}
	}
}