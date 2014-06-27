#include "HoloCodecPassthroughCloud.hpp"

using namespace holo;
using namespace holo::codec;

HoloCodecPassthroughCloud::HoloCodecPassthroughCloud() : isInit_(false)
{

}

HoloCodecPassthroughCloud::~HoloCodecPassthroughCloud()
{


}

bool HoloCodecPassthroughCloud::init(CODEC_TYPE codecType)
{
	codecType_ = codecType;
	isInit_ = true;

	return isInit_;
}


void HoloCodecPassthroughCloud::deinit()
{


}

void HoloCodecPassthroughCloud::encode(boost::shared_ptr<HoloCloud> rawData, boost::shared_ptr<std::vector<unsigned char>>& encodeOut)
{
	encodeOut = boost::shared_ptr<std::vector<unsigned char>>(new std::vector<unsigned char>(rawData->size() * 28));
	memcpy(encodeOut->data(), rawData->points.data(), 28 * rawData->size());
}

void HoloCodecPassthroughCloud::decode(boost::shared_ptr<std::vector<unsigned char>> encodedStream, boost::shared_ptr<HoloCloud>& decodeOut)
{
	decodeOut = boost::shared_ptr<HoloCloud>(new HoloCloud(640,480));
	decodeOut->is_dense = false;
	
	memcpy(decodeOut->points.data(), encodedStream->data(), 28 * decodeOut->size());
}