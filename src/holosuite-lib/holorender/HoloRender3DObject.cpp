#include "HoloRender3DObject.hpp"
#include <boost/asio.hpp>
#include <future>

using namespace holo;
using namespace holo::net;
using namespace holo::render;

HoloRender3DObject::HoloRender3DObject()
{

}

HoloRender3DObject::HoloRender3DObject(const boost::shared_ptr<HoloNetPacket>& objectPacket) :
	objectName_(),
	objectHeader_(),
	objectTransform_(),
	vertSize_(0),
	normalSize_(0),
	colorSize_(0),
	stringSize_(0)
{
	memcpy(&objectHeader_, objectPacket->value.data(), sizeof(objectHeader_));
	objectHeader_.num_vertices = ntohl(objectHeader_.num_vertices);
	objectHeader_.num_points_per_vertex = ntohl(objectHeader_.num_points_per_vertex);
	objectHeader_.num_color_channels = ntohl(objectHeader_.num_color_channels);
	objectHeader_.vertex_stride = ntohl(objectHeader_.vertex_stride);
	objectHeader_.color_stride = ntohl(objectHeader_.color_stride);
	objectHeader_.num_indecies = ntohl(objectHeader_.num_indecies);


	memcpy(&vertSize_, objectPacket->value.data() + sizeof(objectHeader_), sizeof(vertSize_));
	memcpy(&normalSize_, objectPacket->value.data() + sizeof(objectHeader_) + sizeof(vertSize_), sizeof(normalSize_));
	memcpy(&colorSize_, objectPacket->value.data() + sizeof(objectHeader_) + sizeof(vertSize_) + sizeof(normalSize_), sizeof(colorSize_));
	memcpy(&stringSize_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(stringSize_), sizeof(stringSize_));

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
		memcpy(vertices_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_), vertSize_);
		float * realVal = (float*)vertices_;
		for (unsigned int i = 0; i < vertSize_; i++)
		{
			*realVal = ntohf(*realVal);
			realVal++;
		}
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		normals_ = new unsigned char[normalSize_];
		// copy and convert normal floats from network to host endianness
		memcpy(normals_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_, normalSize_);
		float * realVal = (float*)normals_;
		for (unsigned int i = 0; i < normalSize_; i++)
		{
			*realVal = ntohf(*realVal);
			realVal++;
		}
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		colors_ = new unsigned char[colorSize_];
		// copy and convert color floats from network to host endianness
		memcpy(colors_, objectPacket->value.data() + sizeof(objectHeader_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_ + normalSize_, colorSize_);
		float * realVal = (float*)colors_;
		for (unsigned int i = 0; i < colorSize_; i++)
		{
			*realVal = ntohf(*realVal);
			realVal++;
		}
	});

	// create the name string
	objectName_.resize(stringSize_);
	memcpy((void*)objectName_.data(), objectPacket->value.data() + sizeof(objectHeader_)+sizeof(vertSize_)+sizeof(normalSize_)+sizeof(colorSize_)+sizeof(stringSize_)+vertSize_ + normalSize_ + colorSize_, stringSize_);

	// Wait for them to be finished
	vertFuture.wait();
	normFuture.wait();
	colorFuture.wait();
}

const boost::shared_ptr<HoloNetPacket> HoloRender3DObject::CreateNetPacket() const
{
	auto netPacket = boost::shared_ptr<HoloNetPacket>(new HoloNetPacket);

	netPacket->type = HOLO_NET_PACKET_TYPE_OBJECT_ADD;
	netPacket->length = sizeof(objectHeader_) + 3 * sizeof(unsigned int)+vertSize_ + normalSize_ + colorSize_ + vertSize_ + objectName_.size();
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

	// convert size to network byte endianness, and copy to value buffer
	auto vertSize = htonl(vertSize_);
	auto normalSize = htonl(normalSize_);
	auto colorSize = htonl(colorSize_);
	auto stringSize = htonl(stringSize_);

	memcpy(netPacket->value.data() + sizeof(objHeader), &vertSize, sizeof(vertSize));
	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize), &normalSize, sizeof(normalSize));
	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize)+sizeof(normalSize), &colorSize, sizeof(colorSize));
	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize), &stringSize, sizeof(stringSize));

	// Launch vert/normal/color processing simultaneously
	std::future<void> vertFuture =
		std::async(std::launch::async,
		[&]()
	{
		unsigned char *vertices = new unsigned char[vertSize_];
		float *vp = (float*)vertices;

		// copy and convert vertices floats to network endianness
		float * realVal = (float*)vertices_;
		for (unsigned int i = 0; i < vertSize_; i++)
		{
			*vp = htonf(*realVal);
			vp++;
			realVal++;
		}

		memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize), vertices, vertSize_);

		delete[] vertices;
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		unsigned char * normals = new unsigned char[normalSize_];
		float * np = (float*)normals;

		// copy and convert normal floats to network endianness
		float * realVal = (float*)normals;
		for (unsigned int i = 0; i < normalSize_; i++)
		{
			*np = htonf(*realVal);
			np++;
			realVal++;
		}

		memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_, normals, normalSize_);

		delete[] normals;
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		unsigned char * colors = new unsigned char[colorSize_];
		float * cp = (float*)colors;

		// copy and convert color floats from host to network endianness
		float * realVal = (float*)colors_;
		for (unsigned int i = 0; i < colorSize_; i++)
		{
			*cp = htonf(*realVal);
			cp++;
			realVal++;
		}

		memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_ + normalSize_, colors, normalSize_);

		delete[] colors;
	});

	memcpy(netPacket->value.data() + sizeof(objHeader)+sizeof(vertSize)+sizeof(normalSize)+sizeof(colorSize)+sizeof(stringSize)+vertSize_ + normalSize_ + colorSize_, objectName_.data(), objectName_.size());

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

	objectTransform.translate.x = ntohf(objectTransform.translate.x);
	objectTransform.translate.y = ntohf(objectTransform.translate.y);
	objectTransform.translate.z = ntohf(objectTransform.translate.z);

	objectTransform.rotation.x = ntohf(objectTransform.rotation.x);
	objectTransform.rotation.y = ntohf(objectTransform.rotation.y);
	objectTransform.rotation.z = ntohf(objectTransform.rotation.z);
	objectTransform.rotation.w = ntohf(objectTransform.rotation.w);

	objectTransform.scale.x = ntohf(objectTransform.scale.x);
	objectTransform.scale.y = ntohf(objectTransform.scale.y);
	objectTransform.scale.z = ntohf(objectTransform.scale.z);

	objectTransform.bounding_sphere.x = ntohf(objectTransform.bounding_sphere.x);
	objectTransform.bounding_sphere.y = ntohf(objectTransform.bounding_sphere.y);
	objectTransform.bounding_sphere.z = ntohf(objectTransform.bounding_sphere.z);
	objectTransform.bounding_sphere.w = ntohf(objectTransform.bounding_sphere.w);

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

	objectTransform.translate.x = ntohf(objectTransform.translate.x);
	objectTransform.translate.y = ntohf(objectTransform.translate.y);
	objectTransform.translate.z = ntohf(objectTransform.translate.z);

	objectTransform.rotation.x = ntohf(objectTransform.rotation.x);
	objectTransform.rotation.y = ntohf(objectTransform.rotation.y);
	objectTransform.rotation.z = ntohf(objectTransform.rotation.z);
	objectTransform.rotation.w = ntohf(objectTransform.rotation.w);

	objectTransform.scale.x = ntohf(objectTransform.scale.x);
	objectTransform.scale.y = ntohf(objectTransform.scale.y);
	objectTransform.scale.z = ntohf(objectTransform.scale.z);

	objectTransform.bounding_sphere.x = ntohf(objectTransform.bounding_sphere.x);
	objectTransform.bounding_sphere.y = ntohf(objectTransform.bounding_sphere.y);
	objectTransform.bounding_sphere.z = ntohf(objectTransform.bounding_sphere.z);
	objectTransform.bounding_sphere.w = ntohf(objectTransform.bounding_sphere.w);

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