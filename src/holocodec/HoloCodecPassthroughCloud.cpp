#include "HoloCodecPassthroughCloud.hpp"

using namespace holo;
using namespace holo::codec;

HoloCodecPassthroughCloud::HoloCodecPassthroughCloud() : isInit_(false), width_(HOLO_CAPTURE_DEFAULT_Z_WIDTH), height_(HOLO_CAPTURE_DEFAULT_Z_HEIGHT)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.passthroughcloud");
}

HoloCodecPassthroughCloud::HoloCodecPassthroughCloud(int width, int height) : isInit_(false), width_(width), height_(height)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.codec.passthroughcloud");
}

HoloCodecPassthroughCloud::~HoloCodecPassthroughCloud()
{


}

bool HoloCodecPassthroughCloud::init(CODEC_MODE codecMode)
{
	codecMode_ = codecMode;
	isInit_ = true;

	return isInit_;
}


void HoloCodecPassthroughCloud::deinit()
{
	isInit_ = false;
}

void HoloCodecPassthroughCloud::encode(boost::shared_ptr<HoloCloud> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut)
{
	encodeOut = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>(rawData->size() * 28));
	memcpy(encodeOut->data(), rawData->points.data(), 28 * rawData->size());
}

void HoloCodecPassthroughCloud::decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut)
{
	decodeOut = boost::shared_ptr<HoloCloud>(new HoloCloud(width_,height_));
	decodeOut->is_dense = false;
	
	memcpy(decodeOut->points.data(), encodedStream->data(), 28 * decodeOut->size());
}