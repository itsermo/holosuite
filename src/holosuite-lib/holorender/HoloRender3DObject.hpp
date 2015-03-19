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
			HoloRender3DObject(const std::string objectName, unsigned int numIndecies, unsigned int numVertices, float *vertices, float * normals = nullptr, float *colors = nullptr, unsigned int numVertexDimensions = 3, unsigned int numColorChannels = 4);
			HoloRender3DObject(const boost::shared_ptr<holo::net::HoloNetPacket>& objectNetPacket);
			~HoloRender3DObject();

			const HoloObjectHeader& GetObjectInfo() const { return objectHeader_; }
			const std::string& GetObjectName() const { return objectName_; }
			const boost::shared_ptr<holo::net::HoloNetPacket> CreateNetPacket() const;

			void SetTransform(HoloTransform &transform) { objectTransform_ = transform; }
			const HoloTransform& GetTransform() const { return objectTransform_; }

			static const std::tuple<std::string, HoloTransform> GetTransformFromPacket(const boost::shared_ptr<holo::net::HoloNetPacket>& transformNetPacket);
			static const boost::shared_ptr<holo::net::HoloNetPacket> CreateNetPacketFromTransform(const std::tuple<std::string, HoloTransform>& objectTransform);

		private:
			std::string objectName_;
			HoloTransform objectTransform_;
			HoloObjectHeader objectHeader_;

			unsigned int vertSize_;
			unsigned int normalSize_;
			unsigned int colorSize_;
			unsigned int stringSize_;

			void * vertices_;
			void * normals_;
			void * colors_;

		};
		
	}
	
}