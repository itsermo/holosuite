#include "HoloRender3DObject.hpp"
#include <boost/asio.hpp>
#include <future>

using namespace holo;
using namespace holo::net;
using namespace holo::render;

HoloRender3DObject::HoloRender3DObject()
{

}

HoloRender3DObject::HoloRender3DObject(const boost::shared_ptr<HoloNetPacket>& objectPacket)
{
	objectInfo_.info = { 0 };

	memcpy(&objectInfo_.info, objectPacket->value.data(), sizeof(objectInfo_.info));
	objectInfo_.info.num_vertices = ntohl(objectInfo_.info.num_vertices);
	objectInfo_.info.num_points_per_vertex = ntohl(objectInfo_.info.num_points_per_vertex);
	objectInfo_.info.num_color_channels = ntohl(objectInfo_.info.num_color_channels);
	objectInfo_.info.vertex_stride = ntohl(objectInfo_.info.vertex_stride);
	objectInfo_.info.color_stride = ntohl(objectInfo_.info.color_stride);
	objectInfo_.info.num_indecies = ntohl(objectInfo_.info.num_indecies);

	unsigned int vertSize, normalSize, colorSize = 0;
	memcpy(&vertSize, objectPacket->value.data() + sizeof(objectInfo_.info), sizeof(unsigned int));
	memcpy(&normalSize, objectPacket->value.data() + sizeof(objectInfo_.info) + sizeof(unsigned int), sizeof(unsigned int));
	memcpy(&colorSize, objectPacket->value.data() + sizeof(objectInfo_.info) + 2 * sizeof(unsigned int), sizeof(unsigned int));

	vertSize = ntohl(vertSize);
	normalSize = ntohl(normalSize);
	colorSize = ntohl(colorSize);

	// Launch vert/normal/color processing simultaneously
	std::future<void> vertFuture =
		std::async(std::launch::async,
		[&]()
	{
		vertices_ = new unsigned char[vertSize];
		// copy and convert vertices floats from network to host endianness
		memcpy(vertices_, objectPacket->value.data() + sizeof(objectInfo_.info) + 3 * sizeof(unsigned int), sizeof(objectInfo_.info));
		float * realVal = (float*)vertices_;
		for (unsigned int i = 0; i < vertSize; i++)
		{
			*realVal = ntohf(*realVal);
			realVal++;
		}
	});


	std::future<void> normFuture =
		std::async(std::launch::async,
		[&]()
	{
		normals_ = new unsigned char[normalSize];
		// copy and convert normal floats from network to host endianness
		memcpy(normals_, objectPacket->value.data() + sizeof(objectInfo_.info) + 3 * sizeof(unsigned int)+vertSize, sizeof(objectInfo_.info));
		float * realVal = (float*)normals_;
		for (unsigned int i = 0; i < normalSize; i++)
		{
			*realVal = ntohf(*realVal);
			realVal++;
		}
	});

	std::future<void> colorFuture =
		std::async(std::launch::async,
		[&]()
	{
		colors_ = new unsigned char[colorSize];
		// copy and convert color floats from network to host endianness
		memcpy(colors_, objectPacket->value.data() + sizeof(objectInfo_.info) + 3 * sizeof(unsigned int)+ vertSize + normalSize, sizeof(objectInfo_.info));
		float * realVal = (float*)colors_;
		for (unsigned int i = 0; i < colorSize; i++)
		{
			*realVal = ntohf(*realVal);
			realVal++;
		}
	});

	// Wait for them to be finished
	vertFuture.wait();
	normFuture.wait();
	colorFuture.wait();

}

HoloRender3DObject::~HoloRender3DObject()
{
	delete[] colors_;
	delete[] vertices_;
	delete[] normals_;
}