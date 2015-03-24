#include "HoloRender3DObject.hpp"
#include <boost/asio.hpp>
#include <future>
#include "Miniball.hpp"

using namespace holo;
using namespace holo::net;
using namespace holo::render;

HoloRender3DObject::HoloRender3DObject() :
	objectName_(),
	objectHeader_(),
	objectTransform_(),
	vertSize_(0),
	normalSize_(0),
	colorSize_(0),
	stringSize_(0)
{
	objectTransform_.bounding_sphere = {};
	objectTransform_.rotation = {};
	objectTransform_.scale = {};
	objectTransform_.translate = {};
}

HoloRender3DObject::HoloRender3DObject(const std::string objectName, unsigned int numIndecies, unsigned int numVertices, float *vertices, float * normals, float *colors, unsigned int numVertexDimensions, unsigned int numColorChannels) : HoloRender3DObject()
{
	objectName_ = objectName;
	objectHeader_.num_indecies = numIndecies;
	objectHeader_.num_vertices = numVertices;
	objectHeader_.num_color_channels = numColorChannels;
	objectHeader_.num_points_per_vertex = numVertexDimensions;
	objectHeader_.vertex_stride = objectHeader_.num_points_per_vertex * sizeof(float);
	objectHeader_.color_stride = objectHeader_.num_color_channels * sizeof(float);

	// Launch vert/normal/color processing simultaneously
	std::future<void> vertFuture =
		std::async(std::launch::async,
		[&]()
	{
		vertSize_ = objectHeader_.vertex_stride * objectHeader_.num_vertices;
		vertices_ = new unsigned char[vertSize_];
		// copy and vertices floats from object file buffer
		memcpy(vertices_, vertices, vertSize_);
	});

	std::future<void> normalFuture =
		std::async(std::launch::async,
		[&]()
	{
		if (normals)
		{
			normalSize_ = objectHeader_.vertex_stride * objectHeader_.num_vertices;
			normals_ = new unsigned char[normalSize_];
			// copy normals from object file buffer
			memcpy(normals_, normals, normalSize_);
		}
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		if (colors)
		{

			colorSize_ = objectHeader_.color_stride * objectHeader_.num_vertices;
			colors_ = new unsigned char[colorSize_];
			// copy normals from object file buffer
			memcpy(colors_, colors, colorSize_);
		}
	});

	//Use miniball algorithm to get the bounding sphere of the object quickly
	float **ap = new float*[objectHeader_.num_vertices];
	float * pv = vertices;
	for (int i = 0; i < objectHeader_.num_vertices; ++i) {
		ap[i] = pv;
		pv += objectHeader_.num_points_per_vertex;
	}

	// miniball uses a quick method of determining the bounding sphere of all the vertices
	auto miniball3f = Miniball::Miniball<Miniball::CoordAccessor<float**, float*>>(3, (float**)ap, (float**)(ap + numVertices));
	objectTransform_.bounding_sphere.x = miniball3f.center()[0];
	objectTransform_.bounding_sphere.y = miniball3f.center()[1];
	objectTransform_.bounding_sphere.z = miniball3f.center()[2];
	objectTransform_.bounding_sphere.w = miniball3f.squared_radius();

	delete[] ap;

	stringSize_ = objectName_.size();

	vertFuture.wait();
	normalFuture.wait();
	colorFuture.wait();
}

HoloRender3DObject::HoloRender3DObject(const boost::shared_ptr<HoloNetPacket>& objectPacket) : HoloRender3DObject()
{
	memcpy(&objectHeader_, objectPacket->value.data(), sizeof(objectHeader_));
	objectHeader_.num_vertices = ntohl(objectHeader_.num_vertices);
	objectHeader_.num_points_per_vertex = ntohl(objectHeader_.num_points_per_vertex);
	objectHeader_.num_color_channels = ntohl(objectHeader_.num_color_channels);
	objectHeader_.vertex_stride = ntohl(objectHeader_.vertex_stride);
	objectHeader_.color_stride = ntohl(objectHeader_.color_stride);
	objectHeader_.num_indecies = ntohl(objectHeader_.num_indecies);

	memcpy(&objectTransform_, objectPacket->value.data() + sizeof(objectHeader_), sizeof(objectTransform_));

	objectTransform_.translate.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.translate.x));
	objectTransform_.translate.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.translate.y));
	objectTransform_.translate.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.translate.z));

	objectTransform_.rotation.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.rotation.x));
	objectTransform_.rotation.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.rotation.y));
	objectTransform_.rotation.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.rotation.z));
	objectTransform_.rotation.w = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.rotation.w));

	objectTransform_.scale.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.scale.x));
	objectTransform_.scale.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.scale.y));
	objectTransform_.scale.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.scale.z));

	objectTransform_.bounding_sphere.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.x));
	objectTransform_.bounding_sphere.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.y));
	objectTransform_.bounding_sphere.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.z));
	objectTransform_.bounding_sphere.w = ntohf(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.w));

	memcpy(&vertSize_, objectPacket->value.data() + sizeof(objectHeader_) + sizeof(objectTransform_), sizeof(vertSize_));
	memcpy(&normalSize_, objectPacket->value.data() + sizeof(objectHeader_) + sizeof(objectTransform_)+sizeof(vertSize_), sizeof(normalSize_));
	memcpy(&colorSize_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_), sizeof(colorSize_));
	memcpy(&stringSize_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(stringSize_), sizeof(stringSize_));

	vertSize_ = ntohl(vertSize_);
	normalSize_ = ntohl(normalSize_);
	colorSize_ = ntohl(colorSize_);
	stringSize_ = ntohl(stringSize_);

	// Launch vert/normal/color processing simultaneously
	std::future<void> vertFuture =
		std::async(std::launch::async,
		[&]()
	{
		vertices_ = new unsigned char[vertSize_];
		// copy and convert vertices floats from network to host endianness
		memcpy(vertices_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_), vertSize_);
		float * realVal = (float*)vertices_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
		{
			*realVal = ntohf(*(unsigned int*)realVal);
			realVal++;
		}
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		normals_ = new unsigned char[normalSize_];
		// copy and convert normal floats from network to host endianness
		memcpy(normals_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_, normalSize_);
		float * realVal = (float*)normals_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
		{
			*realVal = ntohf(*(unsigned int*)realVal);
			realVal++;
		}
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		colors_ = new unsigned char[colorSize_];
		// copy and convert color floats from network to host endianness
		memcpy(colors_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_ + normalSize_, colorSize_);
		float * realVal = (float*)colors_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_color_channels; i++)
		{
			*realVal = ntohf(*(unsigned int*)realVal);
			realVal++;
		}
	});

	// create the name string
	objectName_.resize(stringSize_);
	memcpy((void*)objectName_.data(), objectPacket->value.data() + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_ + normalSize_ + colorSize_, stringSize_);

	// Wait for them to be finished
	vertFuture.wait();
	normFuture.wait();
	colorFuture.wait();
}

const boost::shared_ptr<HoloNetPacket> HoloRender3DObject::CreateNetPacket() const
{
	auto netPacket = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);

	netPacket->type = HOLO_NET_PACKET_TYPE_OBJECT_ADD;
	netPacket->length = sizeof(objectHeader_) + 4 * sizeof(unsigned int)+vertSize_ + normalSize_ + colorSize_ + vertSize_ + stringSize_;
	netPacket->value.resize(netPacket->length);

	// convert info to network byte-type, and copy to value buffer
	auto objHeader = objectHeader_;
	objHeader.num_vertices = htonl(objHeader.num_vertices);
	objHeader.num_points_per_vertex = htonl(objHeader.num_points_per_vertex);
	objHeader.num_color_channels = htonl(objHeader.num_color_channels);
	objHeader.vertex_stride = htonl(objHeader.vertex_stride);
	objHeader.color_stride = htonl(objHeader.color_stride);
	objHeader.num_indecies = htonl(objHeader.num_indecies);
	memcpy(netPacket->value.data(), &objHeader, sizeof(objHeader));

	auto objectTransform = objectTransform_;
	
	unsigned int x, y, z, w = 0;

	x = htonf(objectTransform.translate.x);
	y = htonf(objectTransform.translate.y);
	z = htonf(objectTransform.translate.z);

	objectTransform.translate.x = reinterpret_cast<float&>(x);
	objectTransform.translate.y = reinterpret_cast<float&>(y);
	objectTransform.translate.z = reinterpret_cast<float&>(z);

	x = htonf(objectTransform.rotation.x);
	y = htonf(objectTransform.rotation.y);
	z = htonf(objectTransform.rotation.z);
	w = htonf(objectTransform.rotation.w);

	objectTransform.rotation.x = reinterpret_cast<float&>(x);
	objectTransform.rotation.y = reinterpret_cast<float&>(y);
	objectTransform.rotation.z = reinterpret_cast<float&>(z);
	objectTransform.rotation.w = reinterpret_cast<float&>(w);

	x = htonf(objectTransform.scale.x);
	y = htonf(objectTransform.scale.y);
	z = htonf(objectTransform.scale.z);

	objectTransform.scale.x = reinterpret_cast<float&>(x);
	objectTransform.scale.y = reinterpret_cast<float&>(y);
	objectTransform.scale.z = reinterpret_cast<float&>(z);
	
	x = htonf(objectTransform.bounding_sphere.x);
	y = htonf(objectTransform.bounding_sphere.y);
	z = htonf(objectTransform.bounding_sphere.z);
	w = htonf(objectTransform.bounding_sphere.w);

	objectTransform.bounding_sphere.x = reinterpret_cast<float&>(x);
	objectTransform.bounding_sphere.y = reinterpret_cast<float&>(y);
	objectTransform.bounding_sphere.z = reinterpret_cast<float&>(z);
	objectTransform.bounding_sphere.w = reinterpret_cast<float&>(w);

	memcpy(netPacket->value.data() + sizeof(objHeader), &objectTransform, sizeof(objectTransform));

	// convert size to network byte endianness, and copy to value buffer
	auto vertSize = htonl(vertSize_);
	auto normalSize = htonl(normalSize_);
	auto colorSize = htonl(colorSize_);
	auto stringSize = htonl(stringSize_);

	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform), &vertSize, sizeof(vertSize));
	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize), &normalSize, sizeof(normalSize));
	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize), &colorSize, sizeof(colorSize));
	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize), &stringSize, sizeof(stringSize));

	// Launch vert/normal/color processing simultaneously
	std::future<void> vertFuture =
		std::async(std::launch::async,
		[&]()
	{
		unsigned char *vertices = new unsigned char[vertSize_];
		unsigned int *vp = (unsigned int*)vertices;

		// copy and convert vertices floats to network endianness
		float * realVal = (float*)vertices_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
		{
			*vp = htonf(*realVal);
			vp++;
			realVal++;
		}

		memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize), vertices, vertSize_);

		delete[] vertices;
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		unsigned char * normals = new unsigned char[normalSize_];
		unsigned int * np = (unsigned int*)normals;

		// copy and convert normal floats to network endianness
		float * realVal = (float*)normals_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
		{
			*np = htonf(*realVal);
			np++;
			realVal++;
		}

		memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_, normals, normalSize_);

		delete[] normals;
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		unsigned char * colors = new unsigned char[colorSize_];
		unsigned int *cp = (unsigned int*)colors;

		// copy and convert color floats from host to network endianness
		float *realVal = (float*)colors_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_color_channels; i++)
		{
			*cp = htonf(*realVal);
			cp++;
			realVal++;
		}

		memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_ + normalSize_, colors, colorSize_);

		delete[] colors;
	});

	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_ + normalSize_ + colorSize_, objectName_.data(), objectName_.size());

	// Wait for them to be finished
	vertFuture.wait();
	normFuture.wait();
	colorFuture.wait();

	return netPacket;

};

const std::tuple<std::string, HoloTransform> HoloRender3DObject::GetTransformFromPacket(const boost::shared_ptr<holo::net::HoloNetPacket>& transformNetPacket)
{
	HoloTransform objectTransform;
	std::string objectName;

	memcpy(&objectTransform, transformNetPacket->value.data(), sizeof(objectTransform));

	objectTransform.translate.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform.translate.x));
	objectTransform.translate.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform.translate.y));
	objectTransform.translate.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform.translate.z));

	objectTransform.rotation.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform.rotation.x));
	objectTransform.rotation.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform.rotation.y));
	objectTransform.rotation.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform.rotation.z));
	objectTransform.rotation.w = ntohf(reinterpret_cast<unsigned int&>(objectTransform.rotation.w));

	objectTransform.scale.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform.scale.x));
	objectTransform.scale.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform.scale.y));
	objectTransform.scale.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform.scale.z));

	objectTransform.bounding_sphere.x = ntohf(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.x));
	objectTransform.bounding_sphere.y = ntohf(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.y));
	objectTransform.bounding_sphere.z = ntohf(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.z));
	objectTransform.bounding_sphere.w = ntohf(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.w));

	unsigned int stringSize = 0;
	memcpy(&stringSize, transformNetPacket->value.data() + sizeof(objectTransform), sizeof(stringSize));
	stringSize = ntohl(stringSize);

	return std::make_tuple(objectName, objectTransform);
}

static const boost::shared_ptr<holo::net::HoloNetPacket> CreateNetPacketFromTransform(const std::tuple<std::string, HoloTransform>& objInfo)
{
	auto netPacket = boost::shared_ptr<holo::net::HoloNetPacket>(new holo::net::HoloNetPacket);
	std::string objectName;
	HoloTransform objectTransform;

	std::tie(objectName, objectTransform) = objInfo;

	unsigned int x, y, z, w = 0;

	x = htonf(objectTransform.translate.x);
	y = htonf(objectTransform.translate.y);
	z = htonf(objectTransform.translate.z);

	objectTransform.translate.x = reinterpret_cast<float&>(x);
	objectTransform.translate.y = reinterpret_cast<float&>(y);
	objectTransform.translate.z = reinterpret_cast<float&>(z);

	x = htonf(objectTransform.rotation.x);
	y = htonf(objectTransform.rotation.y);
	z = htonf(objectTransform.rotation.z);
	w = htonf(objectTransform.rotation.w);

	objectTransform.rotation.x = reinterpret_cast<float&>(x);
	objectTransform.rotation.y = reinterpret_cast<float&>(y);
	objectTransform.rotation.z = reinterpret_cast<float&>(z);
	objectTransform.rotation.w = reinterpret_cast<float&>(w);

	x = htonf(objectTransform.scale.x);
	y = htonf(objectTransform.scale.y);
	z = htonf(objectTransform.scale.z);

	objectTransform.scale.x = reinterpret_cast<float&>(x);
	objectTransform.scale.y = reinterpret_cast<float&>(y);
	objectTransform.scale.z = reinterpret_cast<float&>(z);

	x = htonf(objectTransform.bounding_sphere.x);
	y = htonf(objectTransform.bounding_sphere.y);
	z = htonf(objectTransform.bounding_sphere.z);
	w = htonf(objectTransform.bounding_sphere.w);

	objectTransform.bounding_sphere.x = reinterpret_cast<float&>(x);
	objectTransform.bounding_sphere.y = reinterpret_cast<float&>(y);
	objectTransform.bounding_sphere.z = reinterpret_cast<float&>(z);
	objectTransform.bounding_sphere.w = reinterpret_cast<float&>(w);

	netPacket->type = HOLO_NET_PACKET_TYPE_OBJECT_UPDATE;
	netPacket->length = sizeof(objectTransform)+objectName.size();
	netPacket->value.resize(netPacket->length);

	unsigned int stringSize = ntohl(objectName.size());

	memcpy(netPacket->value.data(), &objectTransform, sizeof(objectTransform));
	memcpy(netPacket->value.data() + sizeof(objectTransform), &stringSize, sizeof(stringSize));
	memcpy(netPacket->value.data() + sizeof(objectTransform)+sizeof(stringSize), objectName.data(), objectName.size());

	return netPacket;
}

HoloRender3DObject::~HoloRender3DObject()
{
	delete[] colors_;
	delete[] vertices_;
	delete[] normals_;
}