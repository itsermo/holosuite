#ifdef ENABLE_HOLO_AUDIO
#include "HoloRenderAudioPortaudio.hpp"

using namespace holo;
using namespace holo::render;

HoloRenderAudioPortaudio::HoloRenderAudioPortaudio() : IHoloRenderAudio(), isInit_(false), audioFormat_(), audioStream_(nullptr), streamParameters_()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.portaudio");

	audioFormat_.depth = HOLO_AUDIO_DEFAULT_FMT_DEPTH;
	audioFormat_.numChannels = HOLO_AUDIO_DEFAULT_FMT_CHAN;
	audioFormat_.frequency = HOLO_AUDIO_DEFAULT_FMT_FREQ;
}

HoloRenderAudioPortaudio::HoloRenderAudioPortaudio(HoloAudioFormat audioFormat) :IHoloRenderAudio(), isInit_(false), audioFormat_(audioFormat), audioStream_(nullptr), streamParameters_()
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.portaudio");
}

HoloRenderAudioPortaudio::~HoloRenderAudioPortaudio()
{
	deinit();
}

//int HoloRenderAudioPortaudio::getHostType()
//{
////#if defined WIN32
////	return paWASAPI;
////#elif defined __APPLE__
////	return paCoreAudio;
////#else
////	return paALSA;
////#endif
//	return Pa_GetDefaultHostApi();
//}

bool HoloRenderAudioPortaudio::init(int which)
{
	if (!isInit_)
	{

		PaError errorCode = paNoError;

		errorCode = Pa_Initialize();
		if (errorCode != paNoError)
		{
			LOG4CXX_ERROR(logger_, "Could not intitalize PortAudio library: " << Pa_GetErrorText(errorCode));
			return false;
		}

		streamParameters_.channelCount = audioFormat_.numChannels;
		streamParameters_.sampleFormat = audioFormat_.depth == 16 ? paInt16 : audioFormat_.depth == 24 ? paInt24 : paInt32;
		streamParameters_.hostApiSpecificStreamInfo = nullptr;

		if (which == -1)
		{
			errorCode = Pa_OpenDefaultStream(&audioStream_, 0, streamParameters_.channelCount, streamParameters_.sampleFormat, (double)audioFormat_.frequency, 16, NULL, NULL);
			if (errorCode != paNoError)
			{
				LOG4CXX_ERROR(logger_, "Could not open audio output stream: " << Pa_GetErrorText(errorCode));
				deinit();
				return false;
			}
		}
		else
		{
			auto devList = this->enumerateDevices();

			if (which >= devList.size())
			{
				LOG4CXX_ERROR(logger_, "Audio input device selection is out of bounds");
				deinit();
				return false;
			}

			int numDevices = Pa_GetDeviceCount();
			int trueDevIndex = -1;

			for (int i = 0; i < numDevices; i++)
			{
				auto devInfo = Pa_GetDeviceInfo(i);

				if (strcmp(devInfo->name, devList[which].c_str()) == 0 && devInfo->hostApi == Pa_GetDefaultHostApi())
				{
					trueDevIndex = i;
					break;
				}

			}

			auto devInfo = Pa_GetDeviceInfo(trueDevIndex);
			streamParameters_.suggestedLatency = devInfo->defaultLowOutputLatency;
			streamParameters_.device = trueDevIndex;

			errorCode = Pa_OpenStream(&audioStream_, NULL, &streamParameters_, (double)audioFormat_.frequency, 0, paNoFlag, NULL, NULL);
			if (errorCode != paNoError)
			{
				LOG4CXX_ERROR(logger_, "Could not open audio output stream: " << Pa_GetErrorText(errorCode));
				deinit();
				return false;
			}
		}


		errorCode = Pa_StartStream(audioStream_);
		if (errorCode != paNoError)
		{
			LOG4CXX_ERROR(logger_, "Could not open audio output stream: " << Pa_GetErrorText(errorCode));
			deinit();
			return false;
		}

		isInit_ = true;
	}
	
	return isInit();
}

void HoloRenderAudioPortaudio::deinit()
{
	if (isInit_)
	{
		PaError errorCode = paNoError;

		errorCode = Pa_StopStream(audioStream_);
		if (errorCode != paNoError)
		{
			LOG4CXX_WARN(logger_, "Could not stop audio output stream: " << Pa_GetErrorText(errorCode));
		}

		errorCode = Pa_CloseStream(audioStream_);
		if (errorCode != paNoError)
		{
			LOG4CXX_WARN(logger_, "Could not close audio output stream: " << Pa_GetErrorText(errorCode));
		}
		audioStream_ = nullptr;

		errorCode = Pa_Terminate();

		if (errorCode != paNoError)
		{
			LOG4CXX_WARN(logger_, "Could not terminate PortAudio library: " << Pa_GetErrorText(errorCode));
		}

	}
}

void HoloRenderAudioPortaudio::playSoundBuffer(boost::shared_ptr<std::vector<uchar>> && soundBuffer)
{
	auto errorCode = Pa_WriteStream(audioStream_, soundBuffer->data(), HOLO_AUDIO_DEFAULT_NUM_FRAMES);
	if (errorCode != paNoError)
	{
		LOG4CXX_WARN(logger_, "Could not write to audio output stream: " << Pa_GetErrorText(errorCode));
	}
}

std::vector<std::string> HoloRenderAudioPortaudio::enumerateDevices()
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

		if (devInfo->maxOutputChannels > 0 && devInfo->hostApi == Pa_GetDefaultHostApi())
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