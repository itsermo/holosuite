#pragma once
#include "../common/CommonDefs.hpp"
#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

namespace holo
{
	namespace render
	{
		class IHoloRenderAudio
		{
		public:
			virtual ~IHoloRenderAudio() = 0;
			virtual bool init(int which) = 0;
			virtual void deinit() = 0;
			virtual void playSoundBuffer(boost::shared_ptr<std::vector<uchar>> && soundBuffer) = 0;
			virtual std::vector<std::string> enumerateDevices() = 0;
			virtual void* getContext() = 0;
			virtual bool isInit() = 0;
		};

		inline IHoloRenderAudio::~IHoloRenderAudio() { }
	}
}