#include <iostream>
#include <boost/program_options.hpp>
#include <log4cxx/logger.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/basicconfigurator.h>

#include <holocapture/HoloCaptureGenerator.hpp>
#include <holoinput/HoloInputDeviceGenerator.hpp>
#include <holorender/HoloRenderGenerator.hpp>
#include <holocodec/HoloCodecGenerator.hpp>
#include <holonet/HoloNetClient.hpp>
#include <holonet/HoloNetServer.hpp>
#include <holosession/HoloSession.hpp>

#include <future>
#include <functional>
#include <thread>

log4cxx::LoggerPtr logger_main(log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite"));

int main(int argc, char *argv[])
{

#ifdef WIN32
	log4cxx::PatternLayoutPtr logLayoutPtr = new log4cxx::PatternLayout(L"%-5p %m%n");
#else
	log4cxx::PatternLayoutPtr logLayoutPtr = new log4cxx::PatternLayout("%-5p %m%n");
#endif

	log4cxx::ConsoleAppenderPtr logAppenderPtr = new log4cxx::ConsoleAppender(logLayoutPtr);
	log4cxx::BasicConfigurator::configure(logAppenderPtr);

	int videoCaptureDevIndex = 0;
	int voxelSize = HOLO_RENDER_DEFAULT_VOXEL_SIZE;
	std::string sessionName;
	std::string remoteAddress;
	holo::HOLO_SESSION_MODE sessionMode;
	boost::filesystem::path filePath;
	holo::capture::CAPTURE_TYPE captureType;
	holo::codec::CODEC_TYPE videoCodecType;
	holo::render::RENDER_TYPE renderType;

	holo::input::INPUT_DEVICE_TYPE inputDeviceType = holo::input::INPUT_DEVICE_TYPE_NONE;
	holo::capture::CAPTURE_AUDIO_TYPE audioCaptureType = holo::capture::CAPTURE_AUDIO_TYPE_NONE;
	holo::codec::CODEC_TYPE audioCodecType = holo::codec::CODEC_TYPE_NONE;

	int audioCaptureDevIndex = 0;
	int audioRenderDevIndex = 0;
	int audioEncoderBitrate = 0;
	holo::render::RENDER_AUDIO_TYPE audioRenderType = holo::render::RENDER_AUDIO_TYPE_NONE;
	holo::HoloAudioFormat audioCaptureFormat = { 0 };
	holo::HoloAudioFormat audioRenderFormat = { 0 };
	bool enableMeshConstruction = HOLO_RENDER_VISUALIZER_DEFAULT_ENABLE_MESH_CONSTRUCTION;
	bool enableVisualFeedback = false;

#ifdef ENABLE_HOLO_DSCP2
	int dscp2HeadNumber = 0;
	std::string dscp2DisplayEnv;
#endif

#ifdef ENABLE_HOLO_DSCP4
	std::string dscp4DisplayEnv;
	algorithm_options_t dscp4AlgorithmOptions = { 0 };
	render_options_t dscp4RenderOptions = { 0 };
	display_options_t dscp4DisplayOptions = { 0 };
	unsigned int dscp4Verbosity = 0;
#endif

	bool enableZSpace = false;

	holo::net::HoloNetProtocolHandshake localInfo = {0};
	holo::net::HoloNetProtocolHandshake infoFromClient = {0};
	holo::net::HoloNetProtocolHandshake infoFromServer = {0};
	holo::capture::HoloCaptureInfo captureInfo = {0};
	holo::codec::HoloCodecOctreeEncodeArgs octreeArgs = {0};
	holo::codec::HoloCodecH264Args h264Args = {0};
	captureInfo.rgbaWidth = HOLO_CAPTURE_DEFAULT_RGB_WIDTH;
	captureInfo.rgbaHeight = HOLO_CAPTURE_DEFAULT_RGB_HEIGHT;
	captureInfo.rgbFPS = HOLO_CAPTURE_DEFAULT_RGB_FPS;
	captureInfo.zWidth = HOLO_CAPTURE_DEFAULT_Z_WIDTH;
	captureInfo.zHeight = HOLO_CAPTURE_DEFAULT_Z_HEIGHT;
	captureInfo.zFPS = HOLO_CAPTURE_DEFAULT_Z_FPS;

	std::unique_ptr<holo::capture::IHoloCapture> videoCapture = nullptr;
	std::unique_ptr<holo::capture::IHoloCaptureAudio> audioCapture = nullptr;
	std::unique_ptr<holo::render::IHoloRenderAudio> audioRenderer = nullptr;
	std::unique_ptr<holo::input::IHoloInputDevice> inputDevice = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> encoderCloud = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> decoderCloud = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> encoderRGBAZ = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> decoderRGBAZ = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<std::vector<unsigned char>>> decoderAudio = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<std::vector<unsigned char>>> encoderAudio = nullptr;
	std::shared_ptr<holo::net::HoloNetClient> client = nullptr;
	std::shared_ptr<holo::net::HoloNetServer> server = nullptr;
	boost::shared_ptr<holo::render::IHoloRender> renderer = nullptr;
	std::unique_ptr<holo::HoloSession> clientSession = nullptr;
	std::unique_ptr<holo::HoloSession> serverSession = nullptr;
	std::unique_ptr<holo::HoloSession> directSession = nullptr;

	// Declare a group of options that will be 
	// allowed only on command line
	boost::program_options::options_description generic_options("Generic options");
	generic_options.add_options()
		("verbosity,v", boost::program_options::value<int>()->default_value(1), "level of detail for console output. valid values are [0-3] from least to most verbose")
		("help,h", "produce help message")
		;

	// Network connection options
	boost::program_options::options_description network_options("Network session options");
	network_options.add_options()
		("name", boost::program_options::value<std::string>()->composing(), "(optional) friendly name of the server or client session")
		("direct", "capture input is directly routed to renderer")
		("server", "start holosuite in server mode, listen for connections")
		("client", boost::program_options::value<std::string>()->composing(), "start holosuite in client mode. (e.g. client=192.168.0.3)")
		("loopback", "capture input is routed through network and to localhost renderer")
		;
    
	// Capture input options
	boost::program_options::options_description capture_options("Capture input source");
	capture_options.add_options()
		("capture-input,i",
			 boost::program_options::value<std::string>()->default_value("openni2"), 
			 "selects the capture method. valid setting is [none,openni2]")
			 ("capture-index", boost::program_options::value<int>()->default_value(0),
			 "(optional) selects which capture device to open")
			 ("capture-width", boost::program_options::value<int>()->default_value(HOLO_CAPTURE_DEFAULT_Z_WIDTH),
			 "(optional) sets the frame buffer width")
			 ("capture-height", boost::program_options::value<int>()->default_value(HOLO_CAPTURE_DEFAULT_Z_HEIGHT),
			 "(optional) sets the frame buffer height")
			 ("capture-fps", boost::program_options::value<float>()->default_value(HOLO_CAPTURE_DEFAULT_Z_FPS),
			 "(optional) sets the capture rate")
		;

#ifdef ENABLE_HOLO_LEAPSDK
	boost::program_options::options_description input_device_options("Audio input source");
	input_device_options.add_options()
		("input-device,n",
		boost::program_options::value<std::string>()->default_value("none"),
		"selects the interaction input method. valid setting is [none,leapsdk]")
		;
#endif

#ifdef ENABLE_HOLO_AUDIO
	// audio input options
	boost::program_options::options_description audio_input_options("Audio input source");
	audio_input_options.add_options()
		("audio-input,a",
		boost::program_options::value<std::string>()->default_value("none"),
		"selects the audio input method. valid setting is [none,portaudio]")
		("audio-input-index", boost::program_options::value<int>()->default_value(0),
		"(optional) selects which audio input device to open")
		("audio-input-freq", boost::program_options::value<int>()->default_value(HOLO_AUDIO_DEFAULT_FMT_FREQ),
		"(optional) sets the sampling rate for audio input")
		("audio-input-num-chan", boost::program_options::value<int>()->default_value(HOLO_AUDIO_DEFAULT_FMT_CHAN),
		"(optional) sets the number of channels [1,2]")
		("audio-input-depth", boost::program_options::value<int>()->default_value(HOLO_AUDIO_DEFAULT_FMT_DEPTH),
		"(optional) sets the bit resolution for audio input")
		;

	// audio encoder options
	boost::program_options::options_description audio_codec_options("Audio encoder");
	audio_codec_options.add_options()
		("audio-encoder,e",
		boost::program_options::value<std::string>()->default_value("none"),
		"selects the audio input method. valid setting is [none,opus]")
		("audio-encoder-bitrate", boost::program_options::value<int>()->default_value(HOLO_AUDIO_DEFAULT_ENCODE_BITRATE),
		"(optional) sets the audio bitrate for encoding")
		;

	// audio output options
	boost::program_options::options_description audio_output_options("Audio output destination");
	audio_output_options.add_options()
		("audio-output,p",
		boost::program_options::value<std::string>()->default_value("none"),
		"selects the audio output method. valid setting is [none,portaudio]")
		("audio-output-index", boost::program_options::value<int>()->default_value(-1),
		"(optional) selects which audio output device to open. -1 selects default audio output.")
		;
#endif



	// Video encoder options
	boost::program_options::options_description codec_options("Video encoder");
	codec_options.add_options()
		("video-encoder,c",
		boost::program_options::value<std::string>()->default_value("h264"),
		"selects which encoder/decoder to use. valid settings are [h264, octree, passthrough-cloud]")
		("h264-settings", boost::program_options::value<std::string>()->multitoken(),
		"h.264 encoder arguments [maxbitrate,gopsize,maxbframes,crf]")
		("h264-z-level", boost::program_options::value<int>()->default_value(HOLO_CODEC_H264_DEFAULT_ZCOMPRESSIONLEVEL),
		"h.264 z image encoder level [0-9]")
		("octree-settings", boost::program_options::value<std::string>()->multitoken(),
		"octree compression settings [showstatistics=0,pointres=0.0035,octreeres=0.001,downsample=1,iframerate=30,encodecolor=1,colorbitres=6]")
		;

	// Render options
	boost::program_options::options_description render_options("Renderer");
	render_options.add_options()
		("render-output,o",
		boost::program_options::value<std::string>()->default_value("visualizer"),
		"valid setting is [visualizer,dscp2,opengl]")
		("render-settings",
		boost::program_options::value<std::string>()->composing(),
		"Render display settings [voxel size, enable fast mesh construction]")
		("visual-feedback",
		boost::program_options::value<bool>()->default_value(false),
		"Show your own capture in on the bottom-left corner")
		;

#ifdef ENABLE_HOLO_DSCP2
	render_options.add_options()("dscp2-head-number", boost::program_options::value<int>()->default_value(0), "DSCP2 instance head number")
		("dscp2-display-env", boost::program_options::value<std::string>()->default_value(":0.0"), "X server display (e.g. \":0.0\")");
#endif

#ifdef ENABLE_HOLO_DSCP4
	render_options.add_options()("dscp4-render-mode", boost::program_options::value<std::string>()->default_value("stereogram"), "valid settings are [modelview,stereogram,aerial,holovideo]")
		("dscp4-display-env", boost::program_options::value<std::string>()->default_value(":0"), "X server display (e.g. \":0.0\")")
		("dscp4-compute-method", boost::program_options::value<std::string>()->default_value("cuda"), "valid options are [cuda,opencl]")
		("dscp4-verbosity", boost::program_options::value<unsigned int>()->default_value(3), "valid options are [0-6]");
#endif

#ifdef ENABLE_HOLO_ZSPACE
	render_options.add_options()("enable-zspace", boost::program_options::value<bool>()->default_value(false), "Enables ZSpace stereo and headtracking (only for use with OpenGL renderer)");
#endif

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic_options).add(network_options).add(capture_options).add(codec_options).add(render_options);

#ifdef ENABLE_HOLO_AUDIO
	cmdline_options.add(audio_input_options).add(audio_codec_options).add(audio_output_options);
#endif

#ifdef ENABLE_HOLO_LEAPSDK
	cmdline_options.add(input_device_options);
#endif

	boost::program_options::variables_map vm;

	try{
		boost::program_options::store(boost::program_options::parse_command_line(argc, argv, cmdline_options), vm);
	}
	catch (std::exception)
	{
		std::cout << "Unknown command line arguments. Valid command line arguments are below." << std::endl << std::endl;
		std::cout << generic_options << std::endl;
		std::cout << network_options << std::endl;
		std::cout << capture_options << std::endl;
		std::cout << codec_options << std::endl;
		std::cout << render_options << std::endl;

#ifdef ENABLE_HOLO_AUDIO
		std::cout << audio_input_options << std::endl;
		std::cout << audio_codec_options << std::endl;
		std::cout << audio_output_options << std::endl;
#endif

		return -1;
	}

	if (vm.count("help") || vm.empty())
	{
		std::cout << "Holosuite is a digital holographic video telepresence software suite." << std::endl << std::endl;
		std::cout << generic_options << std::endl;
		std::cout << network_options << std::endl;
		std::cout << capture_options << std::endl;
		std::cout << codec_options << std::endl;
		std::cout << render_options << std::endl;

#ifdef ENABLE_HOLO_AUDIO
		std::cout << audio_input_options << std::endl;
		std::cout << audio_codec_options << std::endl;
		std::cout << audio_output_options << std::endl;
#endif

		return -1;
	}

	if (vm.count("verbosity"))
	{
		int logLevel = vm["verbosity"].as<int>();

		switch (logLevel)
		{
		case 0:
			logger_main->setLevel(log4cxx::Level::getError());
			break;
		case 1:
			logger_main->setLevel(log4cxx::Level::getInfo());
			break;
		case 2:
			logger_main->setLevel(log4cxx::Level::getDebug());
			break;
		case 3:
			logger_main->setLevel(log4cxx::Level::getAll());
			break;
		default:
			std::cout << "Invalid verbosity setting.  Please choose a setting between 0-3" << std::endl;
			std::cout << generic_options << std::endl;
			return -1;
		}
	}
	else
		logger_main->setLevel(log4cxx::Level::getInfo());

	if (vm.count("name"))
	{
		sessionName = vm["name"].as<std::string>();
		if (sessionName.size() > HOLO_NET_NAME_STR_SIZE)
		{
			std::cout << "Session name is too long.  Please select a session name that is fewer than " << HOLO_NET_NAME_STR_SIZE << " characters." << std::endl << std::endl;
			std::cout << network_options << std::endl;
			return -1;
		}
	}
	else
	{
		sessionName = boost::asio::ip::host_name();
	}

	strcpy((char*)localInfo.clientName, sessionName.c_str());

	if (vm.count("server"))
	{
		//server = std::shared_ptr<holo::net::HoloNetServer>(new holo::net::HoloNetServer());
		sessionMode = holo::HOLO_SESSION_MODE_SERVER;
	}
	else if (vm.count("client"))
	{
		if (vm["client"].as<std::string>().empty())
		{
			std::cout << "Invalid client address.  Please use --client=<HOSTNAME> option to connect to a remote holosuite server." << std::endl << std::endl;
			std::cout << network_options << std::endl;
			return -1;
		}
		else
		{
			//client = std::shared_ptr<holo::net::HoloNetClient>(new holo::net::HoloNetClient());
			remoteAddress = vm["client"].as<std::string>();
			sessionMode = holo::HOLO_SESSION_MODE_CLIENT;
		}
	}
	else if (vm.count("loopback"))
	{
		//client = std::shared_ptr<holo::net::HoloNetClient>(new holo::net::HoloNetClient());
		//server = std::shared_ptr<holo::net::HoloNetServer>(new holo::net::HoloNetServer());
		sessionMode = holo::HOLO_SESSION_MODE_LOOPBACK;
	}
	else if (vm.count("direct"))
	{
		sessionMode = holo::HOLO_SESSION_MODE_DIRECT;
	}
	else
	{
		std::cout << "You must select a session mode by using the \"--server\", \"--client=<HOSTNAME>\", \"--loopback\" or \"--direct\" command line options." << std::endl << std::endl;
		std::cout << network_options << std::endl;
		return -1;
	}

	if (vm.count("capture-input"))
	{
		if (vm.count("capture-index"))
			videoCaptureDevIndex = vm["capture-index"].as<int>();

		if (vm.count("capture-width"))
			captureInfo.rgbaWidth = captureInfo.zWidth = vm["capture-width"].as<int>();

		if (vm.count("capture-height"))
			captureInfo.rgbaHeight = captureInfo.zHeight = vm["capture-height"].as<int>();

		if (vm.count("capture-fps"))
			captureInfo.rgbFPS = captureInfo.zFPS = vm["capture-fps"].as<float>();

		if (vm["capture-input"].as<std::string>().compare("openni2") == 0)
		{
			captureType = holo::capture::CAPTURE_TYPE_OPENNI2;
		}
		else if (vm["capture-input"].as<std::string>().compare("none") == 0)
		{
			captureType = holo::capture::CAPTURE_TYPE_NONE;
		}
		else if (!vm["capture-input"].as<std::string>().empty())
		{
			filePath = boost::filesystem::path(vm["capture-input"].as<std::string>());
			if (boost::filesystem::exists(filePath))
			{
				if (".oni" == boost::filesystem::extension(filePath))
				{
					//capture = holo::capture::HoloCaptureGenerator::fromOpenNI(filePath.string());
					captureType = holo::capture::CAPTURE_TYPE_FILE_ONI;
				}
				else
				{
					std::cout << "Unrecognized file extension \"" << boost::filesystem::extension(filePath) << "\".Please use a valid capture input such as \"-i ~/rgbz-recording.oni\"." << std::endl << std::endl;
					std::cout << capture_options << std::endl;
					return -1;
				}
			}
			else
			{
				std::cout << "File \"" << filePath.string() << "\" not found. Please select a valid filename." << std::endl << std::endl;
				std::cout << capture_options << std::endl;
				return -1;
			}
		}
		else
		{
			std::cout << "Invalid capture-input option selection. Please use a valid capture input such as -i filename.ext" << std::endl << std::endl;
			std::cout << capture_options << std::endl;
			return -1;
		}
	}
	else if (vm.count("loopback"))
	{
		std::cout << "For loopback mode you must select an capture input device by using the --capture-input command line option." << std::endl << std::endl;
		std::cout << capture_options << std::endl;
		return -1;
	}

	if (vm.count("video-encoder"))
	{
		if (vm["video-encoder"].as<std::string>().compare("octree") == 0)
		{
			octreeArgs.showStatistics = HOLO_CODEC_OCTREE_DEFAULT_SHOW_STATISTICS;
			octreeArgs.pointResolution = HOLO_CODEC_OCTREE_DEFAULT_POINT_RESOLUTION;
			octreeArgs.octreeResolution = HOLO_CODEC_OCTREE_DEFAULT_OCTREE_RESOLUTION;
			octreeArgs.doVoxelGridDownsampling = HOLO_CODEC_OCTREE_DEFAULT_DO_VOXEL_GRID_DOWNSAMPLING;
			octreeArgs.frameRate = HOLO_CODEC_OCTREE_DEFAULT_FRAMERATE;
			octreeArgs.doEncodeColorInfo = HOLO_CODEC_OCTREE_DEFAULT_DO_ENCODE_COLOR_INFO;
			octreeArgs.colorBitResolution = HOLO_CODEC_OCTREE_DEFAULT_COLOR_BIT_RESOLUTION;

			if (vm.count("octree-settings"))
			{
				auto octreeSettingsString = vm["octree-settings"].as<std::string>();
				std::vector<std::string> octreeSettings;
				boost::char_separator<char> sep(", ");
				boost::tokenizer<boost::char_separator<char>> tokens(octreeSettingsString, sep);
				for (const auto& t : tokens) {
					octreeSettings.push_back(t);
				}

				if (octreeSettings.size() == 7)
				{
					octreeArgs.showStatistics = static_cast<bool>(atoi(octreeSettings[0].c_str()));
					octreeArgs.pointResolution = atof(octreeSettings[1].c_str());
					octreeArgs.octreeResolution = atof(octreeSettings[2].c_str());
					octreeArgs.doVoxelGridDownsampling = static_cast<bool>(atoi(octreeSettings[3].c_str()));
					octreeArgs.frameRate = atoi(octreeSettings[4].c_str());
					octreeArgs.doEncodeColorInfo = static_cast<bool>(atoi(octreeSettings[5].c_str()));
					octreeArgs.colorBitResolution = atoi(octreeSettings[6].c_str());
				}
				else
				{
					std::cout << "The octree compression settings entered are invalid. Please check the number of arguments and separate values by a comma." << std::endl << std::endl;
					std::cout << codec_options << std::endl;
					return -1;
				}
			}

			videoCodecType = holo::codec::CODEC_TYPE_OCTREE;

		}
		else if (vm["video-encoder"].as<std::string>().compare("passthrough-cloud") == 0)
		{
			videoCodecType = holo::codec::CODEC_TYPE_PASSTHROUGH_CLOUD;
		}
		else if (vm["video-encoder"].as<std::string>().compare("h264") == 0)
		{
			AVRational rat;
			rat.num = HOLO_CODEC_H264_DEFAULT_TIMEBASE_NUM;
			rat.den = static_cast<int>(captureInfo.zFPS);

			h264Args.bitRate = HOLO_CODEC_H264_DEFAULT_BITRATE;
			h264Args.gopSize = HOLO_CODEC_H264_DEFAULT_GOPSIZE;
			h264Args.maxBFrames = HOLO_CODEC_H264_DEFAULT_MAXBFRAMES;
			h264Args.timeBase = rat;
			h264Args.pixelFormat = HOLO_CODEC_H264_DEFAULT_PIXELFMT;
			h264Args.zCompressionLevel = HOLO_CODEC_H264_DEFAULT_ZCOMPRESSIONLEVEL;
			h264Args.crf = HOLO_CODEC_H264_DEFAULT_CRF;

			if (vm.count("h264-settings"))
			{
				auto h264SettingsString = vm["h264-settings"].as<std::string>();
				std::vector<std::string> h264Settings;
				boost::char_separator<char> sep(", ");
				boost::tokenizer<boost::char_separator<char>> tokens(h264SettingsString, sep);
				for (const auto& t : tokens) {
					h264Settings.push_back(t);
				}

				if (h264Settings.size() == 4)
				{
					h264Args.bitRate = atoi(h264Settings[0].c_str());
					h264Args.gopSize = atoi(h264Settings[1].c_str());
					h264Args.maxBFrames = atoi(h264Settings[2].c_str());
					h264Args.crf = atoi(h264Settings[3].c_str());
				}
				else
				{
					std::cout << "The h.264 compression settings entered are invalid. Please check the number of arguments and separate values by a comma." << std::endl << std::endl;
					std::cout << codec_options << std::endl;
					return -1;
				}
			}

			if (vm.count("h264-z-level"))
			{
				h264Args.zCompressionLevel = vm["h264-z-level"].as<int>();
			}

			videoCodecType = holo::codec::CODEC_TYPE_H264;
		}
		else if (vm["video-encoder"].as<std::string>().compare("none") == 0)
		{
			videoCodecType = holo::codec::CODEC_TYPE_NONE;
		}
		else
		{
			std::cout << "Invalid codec selection. Please use a valid codec such as --codec=octree or --codec=h264." << std::endl << std::endl;
			std::cout << codec_options << std::endl;
			return -1;
		}
	}
	else
	{
		std::cout << "You must select a codec type by using the --codec command line option." << std::endl << std::endl;
		std::cout << codec_options << std::endl;
		return -1;
	}

	localInfo.videoCodecType = videoCodecType;

#ifdef ENABLE_HOLO_LEAPSDK
	if (vm.count("input-device"))
	{
		if (vm["input-device"].as<std::string>().compare("leapsdk") == 0)
			inputDeviceType = holo::input::INPUT_DEVICE_TYPE_LEAPSDK;
	}
#endif

#ifdef ENABLE_HOLO_AUDIO

	if (vm.count("audio-input"))
	{
		if (vm["audio-input"].as<std::string>().compare("portaudio") == 0)
		{
			if (vm.count("audio-input-index"))
				audioCaptureDevIndex = vm["audio-input-index"].as<int>();

			if (vm.count("audio-input-num-chan"))
				audioCaptureFormat.numChannels = vm["audio-input-num-chan"].as<int>();

			if (vm.count("audio-input-freq"))
				audioCaptureFormat.frequency = vm["audio-input-freq"].as<int>();

			if (vm.count("audio-input-depth"))
				audioCaptureFormat.depth = vm["audio-input-depth"].as<int>();

			audioCaptureType = holo::capture::CAPTURE_AUDIO_TYPE_PORTAUDIO;
		}
		else if (vm["audio-input"].as<std::string>().compare("none") == 0)
		{
			audioCaptureType = holo::capture::CAPTURE_AUDIO_TYPE_NONE;
		}
		else
		{
			std::cout << "Invalid audio input selection. You must select a valid audio input type [none,portaudio]." << std::endl << std::endl;
			std::cout << audio_input_options << std::endl;
			return -1;
		}
	}
	
	if (vm.count("audio-encoder"))
	{
		if (vm["audio-encoder"].as<std::string>().compare("none") == 0)
		{
			audioCodecType = holo::codec::CODEC_TYPE_NONE;
		}
		else if (vm["audio-encoder"].as<std::string>().compare("opus") == 0)
		{
			audioCodecType = holo::codec::CODEC_TYPE_OPUS;
			if (vm.count("audio-encoder-bitrate"))
			{
				audioEncoderBitrate = vm["audio-encoder-bitrate"].as<int>();
			}
		}
		else
		{
			std::cout << "Invalid audio encoder selection. You must select a valid audio encoder type [none,opus]." << std::endl << std::endl;
			std::cout << audio_codec_options << std::endl;
			return -1;
		}

		localInfo.audioCodecType = audioCodecType;
	}


	if (vm.count("audio-output"))
	{
		if (vm["audio-output"].as<std::string>().compare("none") == 0)
		{
			audioRenderType = holo::render::RENDER_AUDIO_TYPE_NONE;
		}
		else if (vm["audio-output"].as<std::string>().compare("portaudio") == 0)
		{
			audioRenderType = holo::render::RENDER_AUDIO_TYPE_PORTAUDIO;

			if (vm.count("audio-output-index"))
			{
				audioRenderDevIndex = vm["audio-output-index"].as<int>();
			}
		}
		else
		{
			std::cout << "Invalid audio output selection. You must select a valid audio output type [none,portaudio]." << std::endl << std::endl;
			std::cout << audio_output_options << std::endl;
			return -1;
		}
	}
#endif

	if (vm.count("render-output"))
	{
		if (vm.count("render-settings"))
		{
			auto visSettingsString = vm["render-settings"].as<std::string>();
			std::vector<std::string> visSettings;
			boost::char_separator<char> sep(", ");
			boost::tokenizer<boost::char_separator<char>> tokens(visSettingsString, sep);
			for (const auto& t : tokens) {
				visSettings.push_back(t);
			}

			if (visSettings.size() >= 1)
			{
				voxelSize = atoi(visSettings[0].c_str());
				if (visSettings.size() == 2)
					enableMeshConstruction = visSettings[1].compare("true") == 0 ? true : false;
			}
			else
			{
				std::cout << "The visualizer settings entered are invalid. Please check the number of arguments and separate values by a comma." << std::endl << std::endl;
				std::cout << render_options << std::endl;
				return -1;
			}
		}

		if (vm["render-output"].as<std::string>().compare("visualizer") == 0)
		{
			renderType = holo::render::RENDER_TYPE::RENDER_TYPE_VIS3D;
		}
#ifdef ENABLE_HOLO_DSCP2
		else if (vm["render-output"].as<std::string>().compare("dscp2") == 0)
		{
			if (vm.count("dscp2-head-number"))
			{
				dscp2HeadNumber = vm["dscp2-head-number"].as<int>();
			}

			if (vm.count("dscp2-display-env"))
			{
				dscp2DisplayEnv = vm["dscp2-display-env"].as<string>();
			}

			renderType = holo::render::RENDER_TYPE::RENDER_TYPE_DSCP_MKII;
		}
#endif
#ifdef ENABLE_HOLO_DSCP4
		else if (vm["render-output"].as<std::string>().compare("dscp4") == 0)
		{
			dscp4RenderOptions = render_options_t{
				DSCP4_DEFAULT_RENDER_SHADERS_PATH,
				DSCP4_DEFAULT_RENDER_KERNELS_PATH,
				DSCP4_DEFAULT_RENDER_SHADER_FILENAME_PREFIX,
				DSCP4_DEFAULT_RENDER_RENDER_MODE,
				DSCP4_DEFAULT_RENDER_SHADER_MODEL,
				DSCP4_DEFAULT_RENDER_LIGHT_POS_X,
				DSCP4_DEFAULT_RENDER_LIGHT_POS_Y,
				DSCP4_DEFAULT_RENDER_LIGHT_POS_Z,
				DSCP4_DEFAULT_RENDER_AUTOSCALE_ENABLED };

			dscp4AlgorithmOptions = algorithm_options_t{
				DSCP4_DEFAULT_ALGORITHM_NUM_VIEWS_X,
				DSCP4_DEFAULT_ALGORITHM_NUM_VIEWS_Y,
				DSCP4_DEFAULT_ALGORITHM_NUM_WAFELS,
				DSCP4_DEFAULT_ALGORITHM_NUM_SCANLINES,
				DSCP4_DEFAULT_ALGORITHM_FOV_X,
				DSCP4_DEFAULT_ALGORITHM_FOV_Y,
				DSCP4_DEFAULT_ALGORITHM_REF_BEAM_ANGLE,
				DSCP4_DEFAULT_ALGORITHM_TEMP_UPCONVERT_R,
				DSCP4_DEFAULT_ALGORITHM_TEMP_UPCONVERT_G,
				DSCP4_DEFAULT_ALGORITHM_TEMP_UPCONVERT_B,
				DSCP4_DEFAULT_ALGORITHM_WAVELENGTH_R,
				DSCP4_DEFAULT_ALGORITHM_WAVELENGTH_G,
				DSCP4_DEFAULT_ALGORITHM_WAVELENGTH_B,
				DSCP4_DEFAULT_ALGORITHM_GAIN_R,
				DSCP4_DEFAULT_ALGORITHM_GAIN_G,
				DSCP4_DEFAULT_ALGORITHM_GAIN_B,
				DSCP4_DEFAULT_ALGORITHM_Z_NEAR,
				DSCP4_DEFAULT_ALGORITHM_Z_FAR,
				DSCP4_DEFAULT_ALGORITHM_COMPUTE_METHOD,
				DSCP4_DEFAULT_ALGORITHM_OPENCL_KERNEL_FILENAME,
				{ DSCP4_DEFAULT_ALGORITHM_OPENCL_WORKSIZE_X,
				DSCP4_DEFAULT_ALGORITHM_OPENCL_WORKSIZE_Y },
				{ DSCP4_DEFAULT_ALGORITHM_CUDA_BLOCK_DIM_X,
				DSCP4_DEFAULT_ALGORITHM_CUDA_BLOCK_DIM_Y },
				algorithm_cache_t() };

			dscp4DisplayOptions = display_options_t{
				DSCP4_DEFAULT_DISPLAY_NAME,
				DSCP4_DEFAULT_DISPLAY_X11_ENV_VAR,
				DSCP4_DEFAULT_DISPLAY_NUM_HEADS,
				DSCP4_DEFAULT_DISPLAY_NUM_HEADS_PER_GPU,
				DSCP4_DEFAULT_DISPLAY_HEAD_RES_X,
				DSCP4_DEFAULT_DISPLAY_HEAD_RES_Y,
				DSCP4_DEFAULT_DISPLAY_HEAD_RES_X_SPEC,
				DSCP4_DEFAULT_DISPLAY_HEAD_RES_Y_SPEC,
				DSCP4_DEFAULT_DISPLAY_NUM_AOM_CHANNELS,
				DSCP4_DEFAULT_DISPLAY_NUM_SAMPLES_PER_HOLOLINE,
				DSCP4_DEFAULT_DISPLAY_HOLOGRAM_PLANE_WIDTH,
				DSCP4_DEFAULT_DISPLAY_PIXEL_CLOCK_RATE
			};

			if (vm["dscp4-render-mode"].as<std::string>().compare("modelview") == 0)
			{
				dscp4RenderOptions.render_mode = DSCP4_RENDER_MODE_MODEL_VIEWING;
			}
			else if (vm["dscp4-render-mode"].as<std::string>().compare("stereogram") == 0)
			{
				dscp4RenderOptions.render_mode = DSCP4_RENDER_MODE_STEREOGRAM_VIEWING;
			}
			else if (vm["dscp4-render-mode"].as<std::string>().compare("aerial") == 0)
			{
				dscp4RenderOptions.render_mode = DSCP4_RENDER_MODE_AERIAL_DISPLAY;
			}
			else if (vm["dscp4-render-mode"].as<std::string>().compare("holovideo") == 0)
			{
				dscp4RenderOptions.render_mode = DSCP4_RENDER_MODE_HOLOVIDEO_FRINGE;
			}

			dscp4DisplayEnv = vm["dscp4-display-env"].as<string>();
			dscp4DisplayOptions.x11_env_var = dscp4DisplayEnv.c_str();

			if (vm["dscp4-compute-method"].as<std::string>().compare("cuda") == 0)
			{
				dscp4AlgorithmOptions.compute_method = DSCP4_COMPUTE_METHOD_CUDA;
			}
			else if (vm["dscp4-compute-method"].as<std::string>().compare("opencl") == 0)
			{
				dscp4AlgorithmOptions.compute_method = DSCP4_COMPUTE_METHOD_OPENCL;
			}

			if (vm.count("dscp4-verbosity"))
				dscp4Verbosity = vm["dscp4-verbosity"].as<unsigned int>();

			renderType = holo::render::RENDER_TYPE::RENDER_TYPE_DSCP_MKIV;
		}
#endif
		else if (vm["render-output"].as<std::string>().compare("opengl") == 0)
		{
			if (vm.count("enable-zspace"))
				enableZSpace = vm["enable-zspace"].as<bool>();
				
			renderType = holo::render::RENDER_TYPE_OPENGL;
		}
		else if (vm["render-output"].as<std::string>().compare("none") == 0)
		{
			renderType = holo::render::RENDER_TYPE::RENDER_TYPE_NONE;
		}
		else
		{
			std::cout << "Invalid renderer selection. Please use a valid renderer such as --render-output=\"visualizer\"." << std::endl << std::endl;
			std::cout << render_options << std::endl;
			return -1;
		}
	}
	else if (vm.count("loopback"))
	{
		std::cout << "For loopback mode you must select a renderer by using the --render-output command line option." << std::endl << std::endl;
		std::cout << render_options << std::endl;
		return -1;
	}

	if (vm.count("capture-input") == 0 && vm.count("render-output") == 0)
	{
		std::cout << "You must choose either a capture input or renderer to run holosuite in server or client mode." << std::endl << std::endl;
		std::cout << capture_options << std::endl;
		std::cout << render_options << std::endl;
		return -1;
	}

	if (vm.count("visual-feedback") == 0)
	{
		std::cout << "You must choose whether to enable visual feedback or not." << std::endl << std::endl;
		std::cout << render_options << std::endl;
		return -1;
	}
	else
		enableVisualFeedback = vm["visual-feedback"].as<bool>();

	std::cout << "                '" << std::endl;
	std::cout << "               /=\\" << std::endl;
	std::cout << "              /===\\ \\" << std::endl;
	std::cout << "             /=====\\' \\" << std::endl;
	std::cout << "            /=======\\'' \\" << std::endl;
	std::cout << "           /=========\\ ' '\\" << std::endl;
	std::cout << "          /===========\\''   \\" << std::endl;
	std::cout << "         /=============\\ ' '  \\" << std::endl;
	std::cout << "        /===============\\   ''  \\" << std::endl;
	std::cout << "       /=================\\' ' ' ' \\" << std::endl;
	std::cout << "      /===================\\' ' '  ' \\" << std::endl;
	std::cout << "     /=====================\\' '   ' ' \\" << std::endl;
	std::cout << "    /=======================\\  '   ' /" << std::endl;
	std::cout << "   /=========================\\   ' /" << std::endl;
	std::cout << "  /===========================\\'  /" << std::endl;
	std::cout << " /=============================\\/" << std::endl << std::endl;
	std::cout << "            holosuite             " << std::endl << std::endl;

	
	switch (sessionMode)
	{
	case holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_SERVER:
		LOG4CXX_INFO(logger_main, "Holosuite starting in server mode...");
		break;
	case holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_CLIENT:
		LOG4CXX_INFO(logger_main, "Holosuite starting in client mode...");
		break;
	case holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_LOOPBACK:
		LOG4CXX_INFO(logger_main, "Holosuite starting in loopback mode...");
		break;
	case holo::HOLO_SESSION_MODE_DIRECT:
		LOG4CXX_INFO(logger_main, "Holosuite starting in direct mode...");
	default:
		break;
	}

	bool success = false;

	bool shouldConnect = true;
	while (true)
	{
		if (shouldConnect)
		{
			switch (captureType)
			{
			case holo::capture::CAPTURE_TYPE_FILE_PLY:
				//TODO: get PLY file support
				break;
			case holo::capture::CAPTURE_TYPE_FILE_PCD:
				//TODO: get PCD files in
				break;
			case holo::capture::CAPTURE_TYPE_FILE_OBJ:
				//TODO: get OBJ file support
				break;
			case holo::capture::CAPTURE_TYPE_FILE_ONI:
				videoCapture = holo::capture::HoloCaptureGenerator::fromOpenNI2(filePath.string());
				break;
			case holo::capture::CAPTURE_TYPE_OPENNI2:
				videoCapture = holo::capture::HoloCaptureGenerator::fromOpenNI2(captureInfo.rgbaWidth, captureInfo.rgbaHeight, captureInfo.rgbFPS, captureInfo.zWidth, captureInfo.zHeight, captureInfo.zFPS);
				break;
			default:
			case holo::capture::CAPTURE_TYPE_NONE:
				videoCapture = nullptr;
				break;
			}

			if (videoCapture)
			{
				if (!videoCapture->init(videoCaptureDevIndex))
				{
					LOG4CXX_FATAL(logger_main, "Could not open the capture input.  Exiting holosuite...");
					return -1;
				}

				captureInfo = videoCapture->getCaptureInfo();

				localInfo.rgbazWidth = captureInfo.zWidth;
				localInfo.rgbazHeight = captureInfo.zHeight;
				localInfo.captureFPS = captureInfo.zFPS;
				localInfo.captureHOV = captureInfo.rgbHOV;
				localInfo.captureVOV = captureInfo.rgbVOV;

				h264Args.width = captureInfo.zWidth;
				h264Args.height = captureInfo.zHeight;
			}

			switch (inputDeviceType)
			{
			case holo::input::INPUT_DEVICE_TYPE_NONE:
				inputDevice = nullptr;
				break;
#ifdef ENABLE_HOLO_LEAPSDK
			case holo::input::INPUT_DEVICE_TYPE_LEAPSDK:
				inputDevice = holo::input::HoloInputDeviceGenerator::fromLeapSDK();
				break;
#endif
			default:
				break;
			}

			if (inputDevice)
			{
				if (!inputDevice->init())
				{
					LOG4CXX_FATAL(logger_main, "Could not open the interaction input device.  Exiting holosuite...");
					return -1;
				}
			}

			switch (audioCaptureType)
			{
			case holo::capture::CAPTURE_AUDIO_TYPE_NONE:
				audioCapture = nullptr;
				break;
#ifdef ENABLE_HOLO_AUDIO
			case holo::capture::CAPTURE_AUDIO_TYPE_PORTAUDIO:
				audioCapture = holo::capture::HoloCaptureGenerator::fromPortaudio(audioCaptureFormat);
				break;
#endif
			default:
				break;
			}

			if (audioCapture)
			{
				if (!audioCapture->init(audioCaptureDevIndex))
				{
					LOG4CXX_FATAL(logger_main, "Could not open the audio input.  Exiting holosuite...");
					return -1;
				}

				localInfo.audioBitDepth = audioCaptureFormat.depth;
				localInfo.audioFreq = audioCaptureFormat.frequency;
				localInfo.audioNumChan = audioCaptureFormat.numChannels;
			}

			std::future<holo::net::HoloNetProtocolHandshake> serverHandle;

			if (sessionMode == holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_SERVER || sessionMode == holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_LOOPBACK)
			{
				server = std::shared_ptr<holo::net::HoloNetServer>(new holo::net::HoloNetServer);
				//auto serverFunc = std::bind<holo::net::HoloNetProtocolHandshake>(&holo::net::HoloNetServer::listenAndWait, server, HOLO_NET_DEFAULT_PORT, std::placeholders::_1);
				//serverHandle = std::future<holo::net::HoloNetProtocolHandshake>(std::async(std::launch::async, serverFunc, localInfo));
				//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
				server->listenAsync(HOLO_NET_DEFAULT_PORT, localInfo);
			}
			
			if (sessionMode == holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_CLIENT || sessionMode == holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_LOOPBACK)
			{
				client = std::shared_ptr<holo::net::HoloNetClient>(new holo::net::HoloNetClient);

				do{
					try{
						infoFromServer = client->connect(remoteAddress, HOLO_NET_DEFAULT_PORT, localInfo);
					}
					catch (boost::system::system_error error)
					{
						LOG4CXX_INFO(logger_main, "Trying to connect to " << remoteAddress);
						std::this_thread::sleep_for(std::chrono::milliseconds(1000));
					}

				} while (!client->isConnected());
			}
			
			if (sessionMode == holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_SERVER || sessionMode == holo::HOLO_SESSION_MODE::HOLO_SESSION_MODE_LOOPBACK)
				infoFromClient = server->waitForNextClient();

			switch (videoCodecType)
			{
			case holo::codec::CODEC_TYPE_PASSTHROUGH_CLOUD:
					encoderCloud = holo::codec::HoloCodecGenerator::fromPCLPassthrough(localInfo.rgbazWidth, localInfo.rgbazHeight);
				break;
			case holo::codec::CODEC_TYPE_PASSTHROUGH_RGBAZ:
					//TODO: implement passthrough RGBAZ
				break;
			case holo::codec::CODEC_TYPE_OCTREE:
				encoderCloud = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression(octreeArgs);
				break;
			case holo::codec::CODEC_TYPE_H264:
				encoderRGBAZ = holo::codec::HoloCodecGenerator::fromH264(h264Args);
				break;
			case holo::codec::CODEC_TYPE_NONE:
			default:
				encoderCloud = nullptr;
				encoderRGBAZ = nullptr;
				break;
			}

			switch (audioCodecType)
			{
#ifdef ENABLE_HOLO_AUDIO
			case holo::codec::CODEC_TYPE_OPUS:
				encoderAudio = holo::codec::HoloCodecGenerator::fromOpus(audioCaptureFormat, audioEncoderBitrate);
				break;
#endif
			case holo::codec::CODEC_TYPE_NONE:
			default:
				encoderAudio = nullptr;
				break;
			}

			if (encoderCloud)
			{
				if (!encoderCloud->init(holo::codec::CODEC_MODE_ENCODER))
				{
					LOG4CXX_FATAL(logger_main, "Could not initialize the cloud encoder.  Exiting holosuite...");
					return -1;
				}
			}

			if (encoderRGBAZ)
			{
				if (!encoderRGBAZ->init(holo::codec::CODEC_MODE_ENCODER))
				{
					LOG4CXX_FATAL(logger_main, "Could not intitalize the RGBAZ encoder.  Exiting holosuite...");
					return -1;
				}
			}

			if (encoderAudio)
			{
				if (!encoderAudio->init(holo::codec::CODEC_MODE_ENCODER))
				{
					LOG4CXX_FATAL(logger_main, "Could not intitalize the audio encoder.  Exiting holosuite...");
					return -1;
				}
			}

			switch (sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.videoCodecType : infoFromClient.videoCodecType)
			{
			case holo::codec::CODEC_TYPE_PASSTHROUGH_CLOUD:
				decoderCloud = holo::codec::HoloCodecGenerator::fromPCLPassthrough(sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.rgbazWidth : infoFromClient.rgbazWidth, sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.rgbazHeight : infoFromClient.rgbazHeight);
				break;
			case holo::codec::CODEC_TYPE_PASSTHROUGH_RGBAZ:
				//TODO: implement passthrough RGBAZ
				break;
			case holo::codec::CODEC_TYPE_OCTREE:
				decoderCloud = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression(octreeArgs);
				break;
			case holo::codec::CODEC_TYPE_H264:
			{
				holo::codec::HoloCodecH264Args args = { 0 };
				args.width = sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.rgbazWidth : infoFromClient.rgbazWidth;
				args.height = sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.rgbazHeight : infoFromClient.rgbazHeight;
				decoderRGBAZ = holo::codec::HoloCodecGenerator::fromH264(args);
			}
				break;
			case holo::codec::CODEC_TYPE_NONE:
			default:
				decoderCloud = nullptr;
				decoderRGBAZ = nullptr;
				break;
			}

			switch (sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.audioCodecType : infoFromClient.audioCodecType)
			{
#ifdef ENABLE_HOLO_AUDIO
			case holo::codec::CODEC_TYPE_OPUS:
			{
				audioRenderFormat.numChannels = sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.audioNumChan : infoFromClient.audioNumChan;
				audioRenderFormat.frequency = sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.audioFreq : infoFromClient.audioFreq;
				audioRenderFormat.depth = sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? infoFromServer.audioBitDepth : infoFromClient.audioBitDepth;
				decoderAudio = holo::codec::HoloCodecGenerator::fromOpus(audioRenderFormat, 0);
			}
				break;
#endif
			case holo::codec::CODEC_TYPE_NONE:
			default:
				decoderAudio = nullptr;
				break;
			}

			if (decoderCloud)
			{
				if (!decoderCloud->init(holo::codec::CODEC_MODE_DECODER))
				{
					LOG4CXX_FATAL(logger_main, "Could not initialize the cloud decoder.  Exiting holosuite...");
					return -1;
				}
			}

			if (decoderRGBAZ)
			{
				if (!decoderRGBAZ->init(holo::codec::CODEC_MODE_DECODER))
				{
					LOG4CXX_FATAL(logger_main, "Could not intitalize the RGBAZ decoder.  Exiting holosuite...");
					return -1;
				}
			}

			if (decoderAudio)
			{
				if (!decoderAudio->init(holo::codec::CODEC_MODE_DECODER))
				{
					LOG4CXX_FATAL(logger_main, "Could not intitalize the audio decoder.  Exiting holosuite...");
					return -1;
				}
			}

			if (decoderCloud || decoderRGBAZ)
			{
				switch (renderType)
				{
				case holo::render::RENDER_TYPE_VIS2D:
					//TODO: implement 2D VIS
					break;
				case holo::render::RENDER_TYPE_VIS3D:
					renderer = holo::render::HoloRenderGenerator::fromPCLVisualizer(voxelSize, enableMeshConstruction);
					break;
				case holo::render::RENDER_TYPE_OPENGL:
					renderer = holo::render::HoloRenderGenerator::fromOpenGL(voxelSize, enableZSpace);
					break;
#ifdef ENABLE_HOLO_DSCP2
				case holo::render::RENDER_TYPE_DSCP_MKII:
					renderer = holo::render::HoloRenderGenerator::fromDSCP2(dscp2HeadNumber, dscp2DisplayEnv);
#else
					renderer = nullptr;
#endif
					break;

				case holo::render::RENDER_TYPE_DSCP_MKIV:
#ifdef ENABLE_HOLO_DSCP4
					renderer = holo::render::HoloRenderGenerator::fromDSCP4(&dscp4RenderOptions, &dscp4AlgorithmOptions, dscp4DisplayOptions, voxelSize, dscp4Verbosity, (void*)logAppenderPtr);
#else
					renderer = nullptr;
#endif
					break;
				case holo::render::RENDER_TYPE_NONE:
				default:
					renderer = nullptr;
					break;
				}

				if (renderer)
				{
					if (!renderer->init(enableVisualFeedback))
					{
						LOG4CXX_FATAL(logger_main, "Could not intitalize the renderer.  Exiting holosuite...");
						return -1;
					}
				}
			}

			if (decoderAudio)
			{
				switch (audioRenderType)
				{
#ifdef ENABLE_HOLO_AUDIO
				case holo::render::RENDER_AUDIO_TYPE_PORTAUDIO:
					audioRenderer = holo::render::HoloRenderGenerator::fromPortaudio(audioRenderFormat);
					break;
#endif
				case holo::render::RENDER_TYPE_NONE:
				default:
					audioRenderer = nullptr;
					break;
				}


				if (audioRenderer)
				{
					if (!audioRenderer->init(audioRenderDevIndex))
					{
						LOG4CXX_FATAL(logger_main, "Could not intitalize the audio output.  Exiting holosuite...");
						return -1;
					}
				}
			}

			if (sessionMode == holo::HOLO_SESSION_MODE_DIRECT)
			{
				directSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(
					std::move(videoCapture),
					std::move(audioCapture),
					std::move(inputDevice),
					nullptr,
					nullptr,
					nullptr,
					nullptr,
					nullptr,
					nullptr,
					std::move(renderer),
					std::move(audioRenderer),
					nullptr,
					localInfo, enableVisualFeedback
					));
				directSession->start();
			}

			if (sessionMode == holo::HOLO_SESSION_MODE_SERVER || sessionMode == holo::HOLO_SESSION_MODE_LOOPBACK)
			{
				serverSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(
					sessionMode == holo::HOLO_SESSION_MODE_SERVER ? std::move(videoCapture) : nullptr,
					sessionMode == holo::HOLO_SESSION_MODE_SERVER ? std::move(audioCapture) : nullptr,
					sessionMode == holo::HOLO_SESSION_MODE_SERVER ? std::move(inputDevice) : nullptr,
					sessionMode == holo::HOLO_SESSION_MODE_SERVER ? std::move(encoderRGBAZ) : nullptr,
					std::move(decoderRGBAZ),
					sessionMode == holo::HOLO_SESSION_MODE_SERVER ? std::move(encoderCloud) : nullptr,
					std::move(decoderCloud),
					sessionMode == holo::HOLO_SESSION_MODE_SERVER ? std::move(encoderAudio) : nullptr,
					std::move(decoderAudio),
					std::move(renderer),
					std::move(audioRenderer),
					server,
					infoFromClient, enableVisualFeedback
					));
				serverSession->start();
			}

			if (sessionMode == holo::HOLO_SESSION_MODE_CLIENT || sessionMode == holo::HOLO_SESSION_MODE_LOOPBACK)
			{
				clientSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(
					std::move(videoCapture),
					std::move(audioCapture),
					std::move(inputDevice),
					std::move(encoderRGBAZ),
					sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? std::move(decoderRGBAZ) : nullptr,
					std::move(encoderCloud),
					sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? std::move(decoderCloud) : nullptr,
					std::move(encoderAudio),
					sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? std::move(decoderAudio) : nullptr,
					sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? std::move(renderer) : nullptr,
					sessionMode == holo::HOLO_SESSION_MODE_CLIENT ? std::move(audioRenderer) : nullptr,
					client,
					infoFromServer, enableVisualFeedback
					));
				clientSession->start();
			}


			shouldConnect = false;
		}

		if (sessionMode == holo::HOLO_SESSION_MODE_CLIENT)
		{
			if (!clientSession->isRunning())
			{
				shouldConnect = true;
				clientSession.reset();
			}
		}

		if (sessionMode == holo::HOLO_SESSION_MODE_SERVER)
		{
			if (!serverSession->isRunning())
			{
				serverSession.reset();
				shouldConnect = true;
			}
		}


		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	}

	return 0;
}
