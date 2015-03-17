#pragma once

#include <holocommon/CommonDefs.hpp>
#include <holorender/HoloRender3DObject.hpp>
#include <thread>
#include <mutex>
#include <map>
#include <list>

namespace holo
{
	namespace render
	{
		class HoloRenderObjectTracker
		{
		public:
			HoloRenderObjectTracker();
			~HoloRenderObjectTracker();

			void Add3DObject(const boost::shared_ptr<HoloRender3DObject> & object)
			{
				std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
				objectMap_[object->GetObjectName()] = object;
			}

			void Update3DObjectTransform(const std::tuple<std::string, HoloTransform> & objectState)
			{
				std::string objName;
				HoloTransform objTransform;
				std::tie(objName, objTransform) = objectState;

				std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
				auto obj = objectMap_[objName];
				obj->SetTransform(objTransform);
			}

			void Remove3DObject(const std::string & objectName)
			{
				std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
				objectMap_.erase(objectName);
			}

			const std::map<const std::string, boost::shared_ptr<HoloRender3DObject>> Get3DObjects()
			{
				std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
				return objectMap_;
			}

			boost::shared_ptr<holo::net::HoloNetPacket> Create3DObjectsStatePacket()
			{
				std::list<boost::shared_ptr<holo::net::HoloNetPacket>> packetList;
				std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);

			}

		private:
			std::mutex objectMapMutex_;
			std::map<const std::string,boost::shared_ptr<HoloRender3DObject>> objectMap_;

		};
	}
}