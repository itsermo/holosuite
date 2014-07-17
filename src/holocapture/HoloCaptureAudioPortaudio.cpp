#ifdef ENABLE_HOLO_AUDIO
#include "HoloCaptureAudioPortaudio.hpp"

using namespace holo;
using namespace holo::capture;


HoloCaptureAudioPortaudio::HoloCaptureAudioPortaudio() : IHoloCaptureAudio(), isOpen_(false), isInit_(false), audioStream_(nullptr)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.portaudio");
	audioFormat_.depth = HOLO_AUDIO_DEFAULT_FMT_DEPTH;
	audioFormat_.frequency = HOLO_AUDIO_DEFAULT_FMT_FREQ;
	audioFormat_.numChannels = HOLO_AUDIO_DEFAULT_FMT_CHAN;
}

HoloCaptureAudioPortaudio::HoloCaptureAudioPortaudio(HoloAudioFormat audioFormat) : IHoloCaptureAudio(), isOpen_(false), isInit_(false), audioFormat_(audioFormat), audioStream_(nullptr)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.capture.portaudio");
}

HoloCaptureAudioPortaudio::~HoloCaptureAudioPortaudio()
{

}

PaHostApiTypeId HoloCaptureAudioPortaudio::getHostType()
{
#if defined WIN32
	return paWASAPI;
#elif defined __APPLE__
	return paCoreAudio;
#else
	return paALSA;
#endif
}

bool HoloCaptureAudioPortaudio::init(int which)
{
	if (!isInit_)
	{
		auto devList = this->enumerateDevices();

		if (which < 0)
		{
			LOG4CXX_ERROR(logger_, "Audio input device selection index is less than zero");
			return false;
		}

		if (which >= devList.size())
		{
			LOG4CXX_ERROR(logger_, "Audio input device selection is out of bounds")
			return false;
		}

		PaError errorCode = paNoError;
		errorCode = Pa_Initialize();

		if (errorCode != paNoError)
		{
			LOG4CXX_ERROR(logger_, "Could not intitalize PortAudio library: " << Pa_GetErrorText(errorCode));
			return false;
		}

		int numDevices = Pa_GetDeviceCount();
		int trueDevIndex = -1;

		for (int i = 0; i < numDevices; i++)
		{
			auto devInfo = Pa_GetDeviceInfo(i);

			if (strcmp(devInfo->name, devList[which].c_str()) == 0 && devInfo->hostApi == Pa_HostApiTypeIdToHostApiIndex(getHostType()))
			{
				trueDevIndex = i;
				break;
			}

		}

		auto devInfo = Pa_GetDeviceInfo(trueDevIndex);
		streamParameters_.suggestedLatency = devInfo->defaultLowInputLatency;
		streamParameters_.device = trueDevIndex;
		streamParameters_.channelCount = audioFormat_.numChannels;
		streamParameters_.sampleFormat = audioFormat_.depth == 16 ? paInt16 : audioFormat_.depth == 24 ? paInt24 : paInt32;
		streamParameters_.hostApiSpecificStreamInfo = nullptr;

		errorCode = Pa_OpenStream(&audioStream_, &streamParameters_, NULL, (double)audioFormat_.frequency, 0, paNoFlag, NULL, NULL);
		if (errorCode != paNoError)
		{
			LOG4CXX_ERROR(logger_, "Could not open audio input stream: " << Pa_GetErrorText(errorCode));
			deinit();
			return false;
		}

		errorCode = Pa_StartStream(audioStream_);
		if (errorCode != paNoError)
		{
			LOG4CXX_ERROR(logger_, "Could not open audio input stream: " << Pa_GetErrorText(errorCode));
			deinit();
			return false;
		}

		isInit_ = true;
	}

	return isInit_;
}

void HoloCaptureAudioPortaudio::deinit()
{
	if (isInit_)
	{
		PaError errorCode = paNoError;

		errorCode = Pa_StopStream(audioStream_);
		if (errorCode != paNoError)
		{
			LOG4CXX_WARN(logger_, "Could not stop audio input stream: " << Pa_GetErrorText(errorCode));
		}

		errorCode = Pa_CloseStream(audioStream_);
		if (errorCode != paNoError)
		{
			LOG4CXX_WARN(logger_, "Could not close audio input stream: " << Pa_GetErrorText(errorCode));
		}

		errorCode = Pa_Terminate();

		if (errorCode != paNoError)
		{
			LOG4CXX_WARN(logger_, "Could not terminate PortAudio library: " << Pa_GetErrorText(errorCode));
		}
	}
}

void HoloCaptureAudioPortaudio::waitAndGetNextChunk(std::vector<unsigned char>& audioOut)
{
	audioOut.resize(4*HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE * audioFormat_.numChannels * sizeof(short));
	auto errorCode = Pa_ReadStream(audioStream_, audioOut.data(), HOLO_AUDIO_DEFAULT_ENCODE_FRAME_SIZE * 4);
	if (errorCode != paNoError)
		LOG4CXX_WARN(logger_, "Could not get data from audio input device: " << Pa_GetErrorText(errorCode));
}

std::vector<std::string> HoloCaptureAudioPortaudio::enumerateDevices()
{
	std::vector<std::string> devList;
	bool hadToInit = false;
	if (!isInit_)
	{
		PaError errorCode = paNoError;
		errorCode = Pa_Initialize();

		if (errorCode != paNoError)
		{
			auto errorInfo = Pa_GetLastHostErrorInfo();
			LOG4CXX_ERROR(logger_, "Could not intitalize PortAudio library: " << Pa_GetErrorText(errorCode));
			return devList;
		}

		hadToInit = true;
	}

	int numDevices = Pa_GetDeviceCount();

	for (int i = 0; i < numDevices; i++)
	{
		auto devInfo = Pa_GetDeviceInfo(i);

		if (devInfo->maxInputChannels > 0 && devInfo->hostApi == Pa_HostApiTypeIdToHostApiIndex(getHostType()))
		{
			devList.push_back(std::string(devInfo->name));
		}

	}

	if (hadToInit)
	{
		PaError errorCode = paNoError;
		errorCode = Pa_Terminate();

		if (errorCode != paNoError)
		{
			auto errorInfo = Pa_GetLastHostErrorInfo();
			LOG4CXX_WARN(logger_, "Could not terminate PortAudio library: " << Pa_GetErrorText(errorCode));
		}
	}

	return devList;
}


#endif