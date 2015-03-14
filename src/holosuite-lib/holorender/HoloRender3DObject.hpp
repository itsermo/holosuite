#pragma once

#include <holocommon/CommonDefs.hpp>
#include <holonet/HoloNet.hpp>

namespace holo
{
	namespace render
	{
		class HoloRender3DObject
		{

		public:
			HoloRender3DObject();
			HoloRender3DObject(const boost::shared_ptr<HoloNetPacket>& objectPacket);
			~HoloRender3DObject();

			const HoloObject& GetObjectInfo() const { return objectInfo_; }
			const std::string& GetObjectName() const { return objectName_; }

		private:
			std::string objectName_;
			HoloObject objectInfo_;

			void * vertices_;
			void * normals_;
			void * colors_;

		};
		
	}
	
}