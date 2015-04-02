#pragma once

#include <boost/filesystem.hpp>
#include <holocommon/CommonDefs.hpp>
#include <holorender/HoloRender3DObject.hpp>
#include <thread>
#include <mutex>
#include <map>
#include <list>

#ifdef ENABLE_HOLO_ASSIMP
#include <assimp/Importer.hpp>
#endif

namespace holo
{
	namespace render
	{
		class HoloRenderObjectTracker
		{
		public:
			HoloRenderObjectTracker();
			~HoloRenderObjectTracker();

			void Add3DObject(const std::string & fileName);
			void Populate3DObjects(const std::string filePath);
			void Add3DObject(const boost::shared_ptr<HoloRender3DObject> & object);
			void Update3DObjectTransform(const std::tuple<std::string, HoloTransform> & objectState);
			void Remove3DObject(const std::string & objectName);
			void SetObjectOwner(const std::string & objectName, const bool amOwner);

			const std::map<const std::string, boost::shared_ptr<HoloRender3DObject>> Get3DObjects();

			std::list<boost::shared_ptr<holo::net::HoloNetPacket>> Create3DObjectsStatePacketList();

		private:
			std::mutex objectMapMutex_;
			std::map<const std::string,boost::shared_ptr<HoloRender3DObject>> objectMap_;
			log4cxx::LoggerPtr logger_;

#ifdef ENABLE_HOLO_ASSIMP
			Assimp::Importer assetImporter_;
#endif


		};
	}
}