#include "HoloRenderObjectTracker.hpp"

#ifdef ENABLE_HOLO_ASSIMP
#include <assimp/scene.h>           
#include <assimp/postprocess.h>
#include <boost/tokenizer.hpp>
#include <boost/algorithm/string.hpp>
#endif

using namespace holo;
using namespace holo::net;
using namespace holo::render;

HoloRenderObjectTracker::HoloRenderObjectTracker()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.objecttracker");
}

HoloRenderObjectTracker::~HoloRenderObjectTracker()
{

}

void HoloRenderObjectTracker::Add3DObject(const std::string & fileName)
{
#ifdef ENABLE_HOLO_ASSIMP

	unsigned int aiFlags = 0;
	aiFlags |= aiProcess_GenSmoothNormals;
	aiFlags |= aiProcess_Triangulate;

	LOG4CXX_INFO(logger_, "Loading 3D object file \'" << fileName << "\'...")
	auto objectScene = assetImporter_.ReadFile(fileName, aiFlags);

	boost::filesystem::path filePath(fileName);
	std::string meshID = filePath.stem().string();

	if (!objectScene->HasMeshes())
	{
		LOG4CXX_ERROR(logger_, "3D object file '" << fileName << "' does not appear to have any meshes")
		return;
	}

	for (unsigned int m = 0; m < objectScene->mNumMeshes; m++)
	{
		// if it has faces, treat as mesh, otherwise as point cloud
		if (objectScene->mMeshes[m]->HasFaces())
		{
			LOG4CXX_INFO(logger_, meshID << " has " << objectScene->mMeshes[m]->mNumVertices << " vertices")
			if (objectScene->mMeshes[m]->HasNormals())
			{
				LOG4CXX_INFO(logger_, meshID << " has normals")
			}
			else
			{
				LOG4CXX_WARN(logger_, meshID << " does not have normals, lighting effects will look fucked up")
			}

			LOG4CXX_INFO(logger_, meshID << " has " << objectScene->mMeshes[m]->mNumFaces << " faces")

			if (objectScene->mMeshes[m]->HasVertexColors(0))
			{
				LOG4CXX_INFO(logger_, meshID << " has vertex colors")
				
				auto meshObject = boost::shared_ptr<HoloRender3DObject>(new HoloRender3DObject(
				meshID,
				objectScene->mMeshes[m]->mFaces[0].mNumIndices,
				objectScene->mMeshes[m]->mNumVertices,
				(float*)objectScene->mMeshes[m]->mVertices,
				(float*)objectScene->mMeshes[m]->mNormals,
				(float*)objectScene->mMeshes[m]->mColors[0]));

				std::unique_lock<std::mutex> objectMapLock(objectMapMutex_);

				objectMap_[meshID] = meshObject;
			}
			else
			{
				LOG4CXX_WARN(logger_, meshID << " does not have vertex colors--it may look dull")
			
				auto meshObject = boost::shared_ptr<HoloRender3DObject>(new HoloRender3DObject(
				meshID,
				objectScene->mMeshes[m]->mFaces[0].mNumIndices,
				objectScene->mMeshes[m]->mNumVertices,
				(float*)objectScene->mMeshes[m]->mVertices,
				(float*)objectScene->mMeshes[m]->mNormals));

				std::unique_lock<std::mutex> objectMapLock(objectMapMutex_);

				objectMap_[meshID] = meshObject;

			}

		}
		else
		{
			LOG4CXX_DEBUG(logger_, "Found mesh " << m << " with no faces.  Treating vertecies as point cloud")
		}
	}

#else
	LOG4CXX_ERROR(logger_, "Cannot load 3D object file, because holosuite was built without ASSIMP support")
#endif

}

void HoloRenderObjectTracker::SetObjectOwner(const std::string & objectName, bool amOwner)
{
	std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
	auto obj = objectMap_[objectName];

	if (obj)
	{
		if (amOwner != obj->GetAmOwner())
			obj->SetAmOwner(amOwner);
	}

}

void HoloRenderObjectTracker::Populate3DObjects(const std::string filePath)
{
#ifdef ENABLE_HOLO_ASSIMP

	boost::filesystem::path modelsPath(filePath);
	boost::filesystem::directory_iterator end_iter;

	std::string supportedExtStr;
	assetImporter_.GetExtensionList(supportedExtStr);

	std::list<std::string> extensionList;
	boost::replace_all(supportedExtStr, "*", "");

	boost::char_separator<char> sep(";");
	boost::tokenizer<boost::char_separator<char>> tokens(supportedExtStr, sep);

	for (auto&t : tokens)
	{
		std::string ext = t;
		std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
		extensionList.push_back(ext);
	}

	if (boost::filesystem::exists(modelsPath) && boost::filesystem::is_directory(modelsPath))
	{
		for (boost::filesystem::directory_iterator dir_iter(modelsPath); dir_iter != end_iter; ++dir_iter)
		{
			if (boost::filesystem::is_regular_file(dir_iter->status()))
			{
				for (auto &ext : extensionList)
				{
					std::string fileExt = dir_iter->path().extension().string();
					std::transform(fileExt.begin(), fileExt.end(), fileExt.begin(), ::tolower);

					if (fileExt == ext)
						Add3DObject(dir_iter->path().string());
				}
			}
		}
	}

#else
	LOG4CXX_ERROR(logger_, "Cannot load 3D object files, because holosuite was built without ASSIMP support")
#endif
}

void HoloRenderObjectTracker::Add3DObject(const boost::shared_ptr<HoloRender3DObject> & object)
{
	std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
	objectMap_[object->GetObjectName()] = object;
}

void HoloRenderObjectTracker::Update3DObjectTransform(const std::tuple<std::string, HoloTransform> & objectState)
{
	std::string objName;
	HoloTransform objTransform;
	std::tie(objName, objTransform) = objectState;

	std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
	auto obj = objectMap_[objName];
	obj->SetTransform(objTransform);
}

void HoloRenderObjectTracker::Remove3DObject(const std::string & objectName)
{
	std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
	objectMap_.erase(objectName);
}

const std::map<const std::string, boost::shared_ptr<HoloRender3DObject>> HoloRenderObjectTracker::Get3DObjects()
{
	std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);
	return objectMap_;
}

std::list<boost::shared_ptr<holo::net::HoloNetPacket>> HoloRenderObjectTracker::Create3DObjectsStatePacketList()
{
	std::list<boost::shared_ptr<holo::net::HoloNetPacket>> packetList;
	std::lock_guard<std::mutex> objectMapLock(objectMapMutex_);

	for (auto it = objectMap_.begin(); it != objectMap_.end(); it++)
	{
		packetList.push_back(it->second->CreateNetPacket());
	}

	return packetList;
}