#pragma once

#include <holocommon/CommonDefs.hpp>
#include <holonet/HoloNet.hpp>
#include <atomic>

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

			bool GetAmOwner() { return amOwner_.load(); }
			void SetAmOwner(bool amOwner) { amOwner_.store(amOwner); }
			const boost::shared_ptr<holo::net::HoloNetPacket> ToggleOwnerAndGetOwnerChangePacket();
			static const std::tuple<std::string, bool> GetChangeOwnerInfoFromPacket(const boost::shared_ptr<holo::net::HoloNetPacket>& changeOwnerPacket);

			static const std::tuple<std::string, HoloTransform> GetTransformFromPacket(const boost::shared_ptr<holo::net::HoloNetPacket>& transformNetPacket);
			static const boost::shared_ptr<holo::net::HoloNetPacket> CreateNetPacketFromTransform(const std::tuple<std::string, HoloTransform>& objectTransform);
			const float* GetVertexBuffer() const { return (float*)vertices_; }
			const float* GetNormalBuffer() const { return (float*)normals_; }
			const float* GetColorBuffer() const { return (float*)colors_; }

			void SetHasGLBuffers(bool hasGlBuffers) { hasGLBuffers_ = hasGlBuffers; }
			bool GetHasGLBuffers() const { return hasGLBuffers_; }

			void SetGLVertexBufID(unsigned int glVertexBufID) { glVertexBufID_ = glVertexBufID; }
			unsigned int GetGLVertexBufID() const { return glVertexBufID_; }

			void SetGLNormalBufID(unsigned int glNormalBufID) { glNormalBufID_ = glNormalBufID; }
			unsigned int GetGLNormalBufID() const { return glNormalBufID_; }

			void SetGLColorBufID(unsigned int glColorBufID) { glColorBufID_ = glColorBufID; }
			unsigned int GetGLColorBufID() const { return glColorBufID_; }

		private:
			std::string objectName_;
			HoloTransform objectTransform_;
			HoloObjectHeader objectHeader_;

			unsigned int vertSize_;
			unsigned int normalSize_;
			unsigned int colorSize_;
			unsigned int stringSize_;

			unsigned char* vertices_;
			unsigned char* normals_;
			unsigned char* colors_;

			unsigned int glVertexBufID_, glNormalBufID_, glColorBufID_;
			bool hasGLBuffers_;
			std::atomic<bool> amOwner_;
		};
		
	}
	
}
