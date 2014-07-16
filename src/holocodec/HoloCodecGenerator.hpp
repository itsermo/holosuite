#pragma once
#include "IHoloCodec.hpp"
#include "HoloCodecPassthroughCloud.hpp"
#include "HoloCodecOctree.hpp"
#include "HoloCodecH264.hpp"

#ifdef ENABLE_HOLO_AUDIO
#include "HoloCodecOpus.hpp"
#endif

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

			static std::unique_ptr<IHoloCodec<holo::HoloCloud>> fromPCLPassthrough(int width, int height)
			{
				return std::unique_ptr<IHoloCodec<holo::HoloCloud>>(new HoloCodecPassthroughCloud(width, height));
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

			static std::unique_ptr<IHoloCodec<holo::HoloRGBAZMat>> fromH264(HoloCodecH264Args args)
			{
				return std::unique_ptr<IHoloCodec<holo::HoloRGBAZMat>>(new HoloCodecH264(args));
			}

#ifdef ENABLE_HOLO_AUDIO
			static std::unique_ptr<IHoloCodec<std::vector<unsigned char>>> fromOpus()
			{
				return std::unique_ptr<IHoloCodec<std::vector<unsigned char>>>(new HoloCodecOpus());
			}

			static std::unique_ptr<IHoloCodec<std::vector<unsigned char>>> fromOpus(HoloAudioFormat audioFormat, int bitRate)
			{
				return std::unique_ptr<IHoloCodec<std::vector<unsigned char>>>(new HoloCodecOpus(audioFormat, bitRate));
			}
#endif

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