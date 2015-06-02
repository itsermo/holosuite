#include "HoloRender3DObject.hpp"
#include <boost/asio.hpp>
#include <future>
#include "Miniball.hpp"
#include <boost/asio.hpp>

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
	stringSize_(0),
	amOwner_(true),
	hasGLBuffers_(false),
	isLocal_(true)
{
	objectTransform_.bounding_sphere = {};
	objectTransform_.rotation = {};
	objectTransform_.scale = {};
	objectTransform_.translate = {};
}

HoloRender3DObject::HoloRender3DObject(const std::string objectName, unsigned int numIndecies, unsigned int numVertices, float *vertices, float * normals, float *colors, unsigned int numVertexDimensions, unsigned int numColorChannels) : HoloRender3DObject()
{
	isLocal_ = true;

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
		else
			normals_ = nullptr;
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
		else
			colors_ = nullptr;
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

	objectTransform_.scale.x = 1.f;
	objectTransform_.scale.y = 1.f;
	objectTransform_.scale.z = 1.f;

	delete[] ap;

	stringSize_ = objectName_.size();

	vertFuture.wait();
	normalFuture.wait();
	colorFuture.wait();
}

HoloRender3DObject::HoloRender3DObject(const boost::shared_ptr<HoloNetPacket>& objectPacket) : HoloRender3DObject()
{
	amOwner_ = false;
	isLocal_ = false;

	auto packetValue = objectPacket->GetPacketValue();

	memcpy(&objectHeader_, packetValue, sizeof(objectHeader_));
	objectHeader_.num_vertices = boost::asio::detail::socket_ops::network_to_host_long(objectHeader_.num_vertices);
	objectHeader_.num_points_per_vertex = boost::asio::detail::socket_ops::network_to_host_long(objectHeader_.num_points_per_vertex);
	objectHeader_.num_color_channels = boost::asio::detail::socket_ops::network_to_host_long(objectHeader_.num_color_channels);
	objectHeader_.vertex_stride = boost::asio::detail::socket_ops::network_to_host_long(objectHeader_.vertex_stride);
	objectHeader_.color_stride = boost::asio::detail::socket_ops::network_to_host_long(objectHeader_.color_stride);
	objectHeader_.num_indecies = boost::asio::detail::socket_ops::network_to_host_long(objectHeader_.num_indecies);

	memcpy(&objectTransform_, packetValue + sizeof(objectHeader_), sizeof(objectTransform_));

	unsigned int x, y, z, w = 0;

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.translate.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.translate.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.translate.z));

	objectTransform_.translate.x = reinterpret_cast<float&>(x);
	objectTransform_.translate.y = reinterpret_cast<float&>(y);
	objectTransform_.translate.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.rotation.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.rotation.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.rotation.z));
	w = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.rotation.w));

	objectTransform_.rotation.x = reinterpret_cast<float&>(x);
	objectTransform_.rotation.y = reinterpret_cast<float&>(y);
	objectTransform_.rotation.z = reinterpret_cast<float&>(z);
	objectTransform_.rotation.w = reinterpret_cast<float&>(w);

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.scale.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.scale.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.scale.z));

	objectTransform_.scale.x = reinterpret_cast<float&>(x);
	objectTransform_.scale.y = reinterpret_cast<float&>(y);
	objectTransform_.scale.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.z));
	w = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform_.bounding_sphere.w));

	objectTransform_.bounding_sphere.x = reinterpret_cast<float&>(x);
	objectTransform_.bounding_sphere.y = reinterpret_cast<float&>(y);
	objectTransform_.bounding_sphere.z = reinterpret_cast<float&>(z);
	objectTransform_.bounding_sphere.w = reinterpret_cast<float&>(w);

	memcpy(&vertSize_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_), sizeof(vertSize_));
	memcpy(&normalSize_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_), sizeof(normalSize_));
	memcpy(&colorSize_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_), sizeof(colorSize_));
	memcpy(&stringSize_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(stringSize_), sizeof(stringSize_));

	vertSize_ = boost::asio::detail::socket_ops::network_to_host_long(vertSize_);
	normalSize_ = boost::asio::detail::socket_ops::network_to_host_long(normalSize_);
	colorSize_ = boost::asio::detail::socket_ops::network_to_host_long(colorSize_);
	stringSize_ = boost::asio::detail::socket_ops::network_to_host_long(stringSize_);

	// Launch vert/normal/color processing simultaneously
	std::future<void> vertFuture =
		std::async(std::launch::async,
		[&]()
	{
		vertices_ = new unsigned char[vertSize_];
		// copy and convert vertices floats from network to host endianness
		memcpy(vertices_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_), vertSize_);
		float * realVal = (float*)vertices_;
		for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
		{
			unsigned int netVal = boost::asio::detail::socket_ops::network_to_host_long(*reinterpret_cast<unsigned int*>(realVal));
			*realVal = reinterpret_cast<float&>(netVal);
			realVal++;
		}
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		if (normalSize_ > 0)
		{
			normals_ = new unsigned char[normalSize_];
			// copy and convert normal floats from network to host endianness
			memcpy(normals_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_, normalSize_);
			float * realVal = (float*)normals_;
			for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
			{
				unsigned int netVal = boost::asio::detail::socket_ops::network_to_host_long(*reinterpret_cast<unsigned int*>(realVal));
				*realVal = reinterpret_cast<float&>(netVal);
				realVal++;
			}
		}
		else
			normals_ = nullptr;
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		if (colorSize_ > 0)
		{
			colors_ = new unsigned char[colorSize_];
			// copy and convert color floats from network to host endianness
			memcpy(colors_, packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_ + normalSize_, colorSize_);
			float * realVal = (float*)colors_;
			for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_color_channels; i++)
			{
				unsigned int netVal = boost::asio::detail::socket_ops::network_to_host_long(*reinterpret_cast<unsigned int*>(realVal));
				*realVal = reinterpret_cast<float&>(netVal);
				realVal++;
			}
		}
		else
			colors_ = nullptr;
	});

	// create the name string
	objectName_.resize(stringSize_);
	memcpy((void*)objectName_.data(), packetValue + sizeof(objectHeader_)+sizeof(objectTransform_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_ + normalSize_ + colorSize_, stringSize_);

	// Wait for them to be finished
	vertFuture.wait();
	normFuture.wait();
	colorFuture.wait();
}

boost::shared_ptr<HoloRender3DObject> HoloRender3DObject::CreateSimpleObject(const std::string objectName, SIMPLE_OBJECT objectType, float x, float y, float z, float size)
{

	HoloTransform trans = { 0 };
	boost::shared_ptr<HoloRender3DObject> obj;
	unsigned int numIndecies = 0;
	unsigned int numVertices = 0;

	std::vector<float> vertices;
	std::vector<float> normals;
	std::vector<float> colors;

	switch (objectType)
	{
	case SIMPLE_OBJECT_CIRCLE:
		numIndecies = 4;
		numVertices = 4 * 6;
		vertices = {
			1.f, 1.f, -1.f, -1.f, 1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f,		//top panel
			1.f, -1.f, 1.f, -1.f, -1.f, 1.f, -1.f, -1.f, -1.f, 1.f, -1.f, -1.f,	//bottom panel
			1.f, 1.f, 1.f, -1.f, 1.f, 1.f, -1.f, -1.f, 1.f, 1.f, -1.f, 1.f,		//front panel
			1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, 1.f, -1.f, 1.f, 1.f, -1.f,	//back panel
			-1.f, 1.f, 1.f, -1.f, 1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, 1.f,	//left panel
			1.f, 1.f, -1.f, 1.f, 1.f, 1.f, 1.f, -1.f, 1.f, 1.f, -1.f, -1.f			//right panel
		};

		normals = {
			0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f,
			0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f,
			0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f,
			0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f,
			-1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f,
			1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f
		};

		colors = {
			0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f,
			1.f, 0.5f, 0.f, 1.f, 0.5f, 0.f, 1.f, 0.5f, 0.f, 1.f, 0.5f, 0.f,
			1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f,
			1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f,
			0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f,
			1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f
		};

		trans.bounding_sphere.x = 0.f;
		trans.bounding_sphere.y = 0.f;
		trans.bounding_sphere.z = 0.f;
		trans.bounding_sphere.w = 4.f;

		trans.translate.x = x;
		trans.translate.y = y;
		trans.translate.z = z;

		trans.scale.x = size;
		trans.scale.y = size;
		trans.scale.z = 0.f;
		break;
	case SIMPLE_OBJECT_SQUARE:
	{
		numIndecies = 4;
		numVertices = 4 * 6;
		vertices = {
			1.f, 1.f, -1.f, -1.f, 1.f, -1.f, -1.f, 1.f, 1.f, 1.f, 1.f, 1.f,		//top panel
			1.f, -1.f, 1.f, -1.f, -1.f, 1.f, -1.f, -1.f, -1.f, 1.f, -1.f, -1.f,	//bottom panel
			1.f, 1.f, 1.f, -1.f, 1.f, 1.f, -1.f, -1.f, 1.f, 1.f, -1.f, 1.f,		//front panel
			1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, 1.f, -1.f, 1.f, 1.f, -1.f,	//back panel
			-1.f, 1.f, 1.f, -1.f, 1.f, -1.f, -1.f, -1.f, -1.f, -1.f, -1.f, 1.f,	//left panel
			1.f, 1.f, -1.f, 1.f, 1.f, 1.f, 1.f, -1.f, 1.f, 1.f, -1.f, -1.f			//right panel
		};

		normals = {
			0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f,
			0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f,
			0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f,
			0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f,
			-1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f, -1.f, 0.f, 0.f,
			1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f
		};

		colors = {
			0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f,
			1.f, 0.5f, 0.f, 1.f, 0.5f, 0.f, 1.f, 0.5f, 0.f, 1.f, 0.5f, 0.f,
			1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f,
			1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f,
			0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f, 0.f, 0.f, 1.f,
			1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f, 1.f, 0.f, 1.f
		};

		trans.bounding_sphere.x = 0.f;
		trans.bounding_sphere.y = 0.f;
		trans.bounding_sphere.z = 0.f;
		trans.bounding_sphere.w = 4.f;

		trans.translate.x = x;
		trans.translate.y = y;
		trans.translate.z = z;

		trans.scale.x = size;
		trans.scale.y = size;
		trans.scale.z = 0.f;
	}

		break;
	case SIMPLE_OBJECT_TRIANGLE:
	{
		numIndecies = 3;
		numVertices = 6;

		vertices = {
			-0.5f, 0.f, 0.5f,
			0.f, 1.f, 0.5f,
			0.5f, 1.f, 0.5f,
			-0.5f, 0.f, -0.5f,
			0.f, 1.f, -0.5f,
			0.5f, 1.f, -0.5f
		};

		normals = {
			0.f, 0.f, 1.f,
			0.f, 0.f, 1.f,
			0.f, 0.f, 1.f,
			0.f, 0.f, -1.f,
			0.f, 0.f, -1.f,
			0.f, 0.f, -1.f
		};

		colors = {
			1.f,0.f,0.f,
			0.f,1.f,0.f,
			0.f,0.f,1.f,
			1.f, 0.f, 0.f,
			0.f, 1.f, 0.f,
			0.f, 0.f, 1.f
		};

		trans.bounding_sphere.x = 0.f;
		trans.bounding_sphere.y = 0.5f;
		trans.bounding_sphere.z = 0.f;
		trans.bounding_sphere.w = 4.f;

		trans.translate.x = x;
		trans.translate.y = y;
		trans.translate.z = z;

		trans.scale.x = size;
		trans.scale.y = size;
		trans.scale.z = size;
	}

		break;
	case SIMPLE_OBJECT_POINTER:
		numIndecies = 3;
		numVertices = 6;

		vertices = {
			-0.5f, 0.f, 0.5f,
			0.f, 1.f, 0.5f,
			0.5f, 1.f, 0.5f,
			-0.5f, 0.f, -0.5f,
			0.f, 1.f, -0.5f,
			0.5f, 1.f, -0.5f
		};

		normals = {
			0.f, 0.f, 1.f,
			0.f, 0.f, 1.f,
			0.f, 0.f, 1.f,
			0.f, 0.f, -1.f,
			0.f, 0.f, -1.f,
			0.f, 0.f, -1.f
		};

		colors = {
			1.f, 0.f, 0.f,
			0.f, 1.f, 0.f,
			0.f, 0.f, 1.f,
			1.f, 0.f, 0.f,
			0.f, 1.f, 0.f,
			0.f, 0.f, 1.f
		};

		trans.bounding_sphere.x = 0.f;
		trans.bounding_sphere.y = 0.f;
		trans.bounding_sphere.z = 0.f;
		trans.bounding_sphere.w = 4.f;

		trans.translate.x = x;
		trans.translate.y = y;
		trans.translate.z = z;

		trans.scale.x = size;
		trans.scale.y = size;
		trans.scale.z = size;
		break;

	default:
		break;
	}

	obj = boost::shared_ptr<HoloRender3DObject>(new HoloRender3DObject(objectName,
		numIndecies,
		numVertices,
		vertices.data(),
		normals.data(),
		colors.size() > 0 ? colors.data() : nullptr,
		3,
		3));
	obj->SetTransform(trans);

	return obj;
}



const boost::shared_ptr<HoloNetPacket> HoloRender3DObject::CreateNetPacket() const
{
	auto netPacket = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket(HOLO_NET_PACKET_TYPE_OBJECT_ADD, sizeof(objectHeader_)+sizeof(objectTransform_) + 4 * sizeof(unsigned int) + vertSize_ + normalSize_ + colorSize_ + stringSize_));
	auto packetValue = netPacket->GetPacketValue();
	//netPacket->type = HOLO_NET_PACKET_TYPE_OBJECT_ADD;
	//netPacket->length = sizeof(objectHeader_) + 4 * sizeof(unsigned int)+vertSize_ + normalSize_ + colorSize_ + vertSize_ + stringSize_;
	//netPacket->value.resize(netPacket->length);

	// convert info to network byte-type, and copy to value buffer
	auto objHeader = objectHeader_;
	objHeader.num_vertices = boost::asio::detail::socket_ops::host_to_network_long(objHeader.num_vertices);
	objHeader.num_points_per_vertex = boost::asio::detail::socket_ops::host_to_network_long(objHeader.num_points_per_vertex);
	objHeader.num_color_channels = boost::asio::detail::socket_ops::host_to_network_long(objHeader.num_color_channels);
	objHeader.vertex_stride = boost::asio::detail::socket_ops::host_to_network_long(objHeader.vertex_stride);
	objHeader.color_stride = boost::asio::detail::socket_ops::host_to_network_long(objHeader.color_stride);
	objHeader.num_indecies = boost::asio::detail::socket_ops::host_to_network_long(objHeader.num_indecies);
	memcpy(packetValue, &objHeader, sizeof(objHeader));

	auto objectTransform = objectTransform_;
	
	unsigned int x, y, z, w = 0;

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.translate.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.translate.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.translate.z));

	objectTransform.translate.x = reinterpret_cast<float&>(x);
	objectTransform.translate.y = reinterpret_cast<float&>(y);
	objectTransform.translate.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.z));
	w = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.w));

	objectTransform.rotation.x = reinterpret_cast<float&>(x);
	objectTransform.rotation.y = reinterpret_cast<float&>(y);
	objectTransform.rotation.z = reinterpret_cast<float&>(z);
	objectTransform.rotation.w = reinterpret_cast<float&>(w);

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.scale.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.scale.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.scale.z));

	objectTransform.scale.x = reinterpret_cast<float&>(x);
	objectTransform.scale.y = reinterpret_cast<float&>(y);
	objectTransform.scale.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.z));
	w = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.w));

	objectTransform.bounding_sphere.x = reinterpret_cast<float&>(x);
	objectTransform.bounding_sphere.y = reinterpret_cast<float&>(y);
	objectTransform.bounding_sphere.z = reinterpret_cast<float&>(z);
	objectTransform.bounding_sphere.w = reinterpret_cast<float&>(w);

	memcpy(packetValue + sizeof(objHeader), &objectTransform, sizeof(objectTransform));

	// convert size to network byte endianness, and copy to value buffer
	auto vertSize = boost::asio::detail::socket_ops::host_to_network_long(vertSize_);
	auto normalSize = boost::asio::detail::socket_ops::host_to_network_long(normalSize_);
	auto colorSize = boost::asio::detail::socket_ops::host_to_network_long(colorSize_);
	auto stringSize = boost::asio::detail::socket_ops::host_to_network_long(stringSize_);

	memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform), &vertSize, sizeof(vertSize));
	memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize), &normalSize, sizeof(normalSize));
	memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize), &colorSize, sizeof(colorSize));
	memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize), &stringSize, sizeof(stringSize));

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
			*vp = boost::asio::detail::socket_ops::host_to_network_long(*reinterpret_cast<unsigned int*>(realVal));
			vp++;
			realVal++;
		}

		memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize), vertices, vertSize_);

		delete[] vertices;
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		if (normalSize_ > 0)
		{

			unsigned char * normals = new unsigned char[normalSize_];
			unsigned int * np = (unsigned int*)normals;

			// copy and convert normal floats to network endianness
			float * realVal = (float*)normals_;
			for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_points_per_vertex; i++)
			{
				*np = boost::asio::detail::socket_ops::host_to_network_long(*reinterpret_cast<unsigned int*>(realVal));
				np++;
				realVal++;
			}

			memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_, normals, normalSize_);

			delete[] normals;
		}
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		if (colorSize_ > 0)
		{
			unsigned char * colors = new unsigned char[colorSize_];
			unsigned int *cp = (unsigned int*)colors;

			// copy and convert color floats from host to network endianness
			float *realVal = (float*)colors_;
			for (unsigned int i = 0; i < objectHeader_.num_vertices * objectHeader_.num_color_channels; i++)
			{
				*cp = boost::asio::detail::socket_ops::host_to_network_long(*reinterpret_cast<unsigned int*>(realVal));
				cp++;
				realVal++;
			}

			memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_ + normalSize_, colors, colorSize_);

			delete[] colors;
		}
	});

	memcpy(packetValue + sizeof(objHeader)+sizeof(objectTransform)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_ + normalSize_ + colorSize_, objectName_.data(), objectName_.size());

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

	auto packetValue = transformNetPacket->GetPacketValue();

	memcpy(&objectTransform, packetValue, sizeof(objectTransform));

	unsigned int x, y, z, w = 0;

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.translate.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.translate.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.translate.z));

	objectTransform.translate.x = reinterpret_cast<float&>(x);
	objectTransform.translate.y = reinterpret_cast<float&>(y);
	objectTransform.translate.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.z));
	w = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.w));

	objectTransform.rotation.x = reinterpret_cast<float&>(x);
	objectTransform.rotation.y = reinterpret_cast<float&>(y);
	objectTransform.rotation.z = reinterpret_cast<float&>(z);
	objectTransform.rotation.w = reinterpret_cast<float&>(w);

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.scale.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.scale.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.scale.z));

	objectTransform.scale.x = reinterpret_cast<float&>(x);
	objectTransform.scale.y = reinterpret_cast<float&>(y);
	objectTransform.scale.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.x));
	y = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.y));
	z = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.z));
	w = boost::asio::detail::socket_ops::network_to_host_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.w));

	objectTransform.bounding_sphere.x = reinterpret_cast<float&>(x);
	objectTransform.bounding_sphere.y = reinterpret_cast<float&>(y);
	objectTransform.bounding_sphere.z = reinterpret_cast<float&>(z);
	objectTransform.bounding_sphere.w = reinterpret_cast<float&>(w);

	unsigned int stringSize = 0;
	memcpy(&stringSize, packetValue + sizeof(objectTransform), sizeof(stringSize));
	stringSize = boost::asio::detail::socket_ops::network_to_host_long(stringSize);
	
	objectName.resize(stringSize);
	memcpy((void*)objectName.data(), packetValue + sizeof(objectTransform)+sizeof(stringSize), stringSize);

	return std::make_tuple(objectName, objectTransform);
}

const boost::shared_ptr<holo::net::HoloNetPacket> HoloRender3DObject::CreateNetPacketFromTransform(const std::tuple<std::string, HoloTransform>& objInfo)
{
	
	std::string objectName;
	HoloTransform objectTransform;

	std::tie(objectName, objectTransform) = objInfo;

	unsigned int x, y, z, w = 0;

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.translate.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.translate.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.translate.z));

	objectTransform.translate.x = reinterpret_cast<float&>(x);
	objectTransform.translate.y = reinterpret_cast<float&>(y);
	objectTransform.translate.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.z));
	w = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.rotation.w));

	objectTransform.rotation.x = reinterpret_cast<float&>(x);
	objectTransform.rotation.y = reinterpret_cast<float&>(y);
	objectTransform.rotation.z = reinterpret_cast<float&>(z);
	objectTransform.rotation.w = reinterpret_cast<float&>(w);

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.scale.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.scale.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.scale.z));

	objectTransform.scale.x = reinterpret_cast<float&>(x);
	objectTransform.scale.y = reinterpret_cast<float&>(y);
	objectTransform.scale.z = reinterpret_cast<float&>(z);

	x = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.x));
	y = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.y));
	z = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.z));
	w = boost::asio::detail::socket_ops::host_to_network_long(reinterpret_cast<unsigned int&>(objectTransform.bounding_sphere.w));

	objectTransform.bounding_sphere.x = reinterpret_cast<float&>(x);
	objectTransform.bounding_sphere.y = reinterpret_cast<float&>(y);
	objectTransform.bounding_sphere.z = reinterpret_cast<float&>(z);
	objectTransform.bounding_sphere.w = reinterpret_cast<float&>(w);

	unsigned int stringSize = boost::asio::detail::socket_ops::network_to_host_long(objectName.size());

	auto netPacket = boost::shared_ptr<holo::net::HoloNetPacket>(new holo::net::HoloNetPacket(HOLO_NET_PACKET_TYPE_OBJECT_UPDATE, sizeof(objectTransform)+sizeof(stringSize)+objectName.size()));
	//netPacket->type = HOLO_NET_PACKET_TYPE_OBJECT_UPDATE;
	//netPacket->length = sizeof(objectTransform)+sizeof(stringSize)+objectName.size();
	//netPacket->value.resize(netPacket->length);
	auto packetValue = netPacket->GetPacketValue();
	memcpy(packetValue, &objectTransform, sizeof(objectTransform));
	memcpy(packetValue + sizeof(objectTransform), &stringSize, sizeof(stringSize));
	memcpy(packetValue + sizeof(objectTransform)+sizeof(stringSize), objectName.data(), objectName.size());

	return netPacket;
}

const boost::shared_ptr<holo::net::HoloNetPacket> HoloRender3DObject::ToggleOwnerAndGetOwnerChangePacket()
{
	auto ownerChangePacket = boost::shared_ptr<holo::net::HoloNetPacket>(new HoloNetPacket(holo::net::HOLO_NET_PACKET_TYPE_OBJECT_CHANGE_OWNER, 2 * sizeof(unsigned int)+stringSize_));
	auto packetValue = ownerChangePacket->GetPacketValue();
	//ownerChangePacket->type = holo::net::HOLO_NET_PACKET_TYPE_OBJECT_CHANGE_OWNER;
	//ownerChangePacket->length = 2*sizeof(unsigned int)+stringSize_;
	//ownerChangePacket->value.resize(ownerChangePacket->length);

	unsigned int amOwnerInt = static_cast<unsigned int>(amOwner_.load());
	amOwnerInt = boost::asio::detail::socket_ops::host_to_network_long(amOwnerInt);
	unsigned int stringSize = boost::asio::detail::socket_ops::host_to_network_long(stringSize_);

	memcpy(packetValue, &amOwnerInt, sizeof(amOwnerInt));
	memcpy(packetValue + sizeof(amOwnerInt), &stringSize, sizeof(stringSize));
	memcpy(packetValue + sizeof(amOwnerInt)+sizeof(stringSize), objectName_.data(), stringSize_);

	amOwner_.store(!amOwner_.load());

	return ownerChangePacket;
}

const std::tuple<std::string, bool> HoloRender3DObject::GetChangeOwnerInfoFromPacket(const boost::shared_ptr<holo::net::HoloNetPacket>& changeOwnerPacket)
{
	unsigned int amOwnerInt = 2;
	unsigned int stringSize = 0;
	std::string objectName;

	auto packetValue = changeOwnerPacket->GetPacketValue();

	memcpy(&amOwnerInt, packetValue, sizeof(amOwnerInt));
	memcpy(&stringSize, packetValue + sizeof(amOwnerInt), sizeof(stringSize));

	stringSize = boost::asio::detail::socket_ops::network_to_host_long(stringSize);

	objectName.resize(stringSize);
	
	memcpy((void*)objectName.data(), packetValue + sizeof(amOwnerInt)+sizeof(stringSize), stringSize);

	return std::make_tuple(objectName, amOwnerInt);
}

HoloRender3DObject::~HoloRender3DObject()
{
	delete[] colors_;
	delete[] vertices_;
	delete[] normals_;
}
