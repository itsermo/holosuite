#pragma once

#include "../holocommon/CommonDefs.hpp"

namespace holo
{
	namespace capture
	{
		struct HoloCaptureInfo
		{
			uint32_t rgbaWidth;
			uint32_t rgbaHeight;
			uint32_t zWidth;
			uint32_t zHeight;
			float rgbFPS;
			float zFPS;
			float rgbHOV;
			float rgbVOV;
			float zHOV;
			float zVOV;
		};
	}
}
