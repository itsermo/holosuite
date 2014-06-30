#include <iostream>
#include <boost/program_options.hpp>
#include <log4cxx/logger.h>
#include <log4cxx/patternlayout.h>
#include <log4cxx/consoleappender.h>
#include <log4cxx/basicconfigurator.h>

#include "../holocapture/HoloCaptureGenerator.hpp"
#include "../holorender/HoloRenderGenerator.hpp"
#include "../holocodec/HoloCodecGenerator.hpp"
#include "../holonet/HoloNetClient.hpp"
#include "../holonet/HoloNetServer.hpp"
#include "HoloSession.hpp"

#include <future>
#include <functional>
#include <thread>


log4cxx::LoggerPtr logger_main(log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite"));


enum HOLO_SESSION_MODE
{
	HOLO_SESSION_MODE_SERVER,
	HOLO_SESSION_MODE_CLIENT,
	HOLO_SESSION_MODE_FEEDBACK
};


int main(int argc, char *argv[])
{

#if (WIN32)
	log4cxx::PatternLayoutPtr logLayoutPtr = new log4cxx::PatternLayout(L"%-5p %m%n");
#else
	log4cxx::PatternLayoutPtr logLayoutPtr = new log4cxx::PatternLayout("%-5p %m%n");
#endif

	log4cxx::ConsoleAppenderPtr logAppenderPtr = new log4cxx::ConsoleAppender(logLayoutPtr);
	log4cxx::BasicConfigurator::configure(logAppenderPtr);

	int captureIndex = 0;

	std::string sessionName;
	std::string remoteAddress;
	HOLO_SESSION_MODE sessionMode;
	holo::net::HoloNetProtocolHandshake localInfo;
	holo::net::HoloNetProtocolHandshake infoFromClient;
	holo::net::HoloNetProtocolHandshake infoFromServer;
	holo::capture::HoloCaptureInfo captureInfo = {0};
	captureInfo.rgbaWidth = HOLO_CAPTURE_DEFAULT_RGB_WIDTH;
	captureInfo.rgbaHeight = HOLO_CAPTURE_DEFAULT_RGB_HEIGHT;
	captureInfo.rgbFPS = HOLO_CAPTURE_DEFAULT_RGB_FPS;
	captureInfo.zWidth = HOLO_CAPTURE_DEFAULT_Z_WIDTH;
	captureInfo.zHeight = HOLO_CAPTURE_DEFAULT_Z_HEIGHT;
	captureInfo.zFPS = HOLO_CAPTURE_DEFAULT_Z_FPS;

	std::unique_ptr<holo::capture::IHoloCapture> capture = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> codecClient = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloCloud>> codecServer = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> codecClientRGBAZ = nullptr;
	std::unique_ptr<holo::codec::IHoloCodec<holo::HoloRGBAZMat>> codecServerRGBAZ = nullptr;
	std::shared_ptr<holo::net::HoloNetClient> client = nullptr;
	std::shared_ptr<holo::net::HoloNetServer> server = nullptr;
	std::unique_ptr<holo::render::IHoloRender> renderer = nullptr;
	std::unique_ptr<holo::HoloSession> clientSession = nullptr;
	std::unique_ptr<holo::HoloSession> serverSession = nullptr;

	// Declare a group of options that will be 
	// allowed only on command line
	boost::program_options::options_description generic_options("Generic options");
	generic_options.add_options()
		("verbosity,v", boost::program_options::value<int>()->default_value(1), "level of detail for console output. valid values are [0-4] from least to most verbose")
		("help,h", "produce help message")
		;

	// Network connection options
	boost::program_options::options_description network_options("Network session options");
	network_options.add_options()
		("name", boost::program_options::value<std::string>()->composing(), "(optional) friendly name of the server or client session")
		("server", "start holosuite in server mode, listen for connections")
		("client", boost::program_options::value<std::string>()->composing(), "start holosuite in client mode. (e.g. client=192.168.0.3)")
		("feedback", "capture input is routed to local renderer")
		;
    
	// Capture input options
	boost::program_options::options_description capture_options("Capture input source");
	capture_options.add_options()
		("capture-input,i",
			 boost::program_options::value<std::string>()->default_value("openni2"), 
			 "selects the capture method. valid setting is [openni2]")
			 ("capture-index", boost::program_options::value<int>()->default_value(0),
			 "(optional) selects which capture device to open")
			 ("capture-width", boost::program_options::value<int>()->default_value(HOLO_CAPTURE_DEFAULT_Z_WIDTH),
			 "(optional) sets the frame buffer width")
			 ("capture-height", boost::program_options::value<int>()->default_value(HOLO_CAPTURE_DEFAULT_Z_HEIGHT),
			 "(optional) sets the frame buffer height")
			 ("capture-fps", boost::program_options::value<float>()->default_value(HOLO_CAPTURE_DEFAULT_Z_FPS),
			 "(optional) sets the capture rate")
		;

	// Codec options
	boost::program_options::options_description codec_options("Codec");
	codec_options.add_options()
		("codec,c",
		boost::program_options::value<std::string>()->default_value("h264"),
		"selects which encoder/decoder to use. valid settings are [h264, octree, passthrough-cloud]")
		("h264-settings", boost::program_options::value<std::string>()->multitoken(),
		"h.264 encoder arguments [bitrate,gopsize,maxbframes]")
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
		"valid setting is [visualizer]")
		("visualizer-settings",
		boost::program_options::value<std::string>()->composing(),
		"PCL visualizer settings [voxel size]")
		;

	boost::program_options::options_description cmdline_options;
	cmdline_options.add(generic_options).add(network_options).add(capture_options).add(codec_options).add(render_options);

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

	if (vm.count("server"))
	{
		server = std::shared_ptr<holo::net::HoloNetServer>(new holo::net::HoloNetServer());
		sessionMode = HOLO_SESSION_MODE_SERVER;
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
			client = std::shared_ptr<holo::net::HoloNetClient>(new holo::net::HoloNetClient());
			remoteAddress = vm["client"].as<std::string>();
			sessionMode = HOLO_SESSION_MODE_CLIENT;
		}
	}
	else if (vm.count("feedback"))
	{
		client = std::shared_ptr<holo::net::HoloNetClient>(new holo::net::HoloNetClient());
		server = std::shared_ptr<holo::net::HoloNetServer>(new holo::net::HoloNetServer());
		sessionMode = HOLO_SESSION_MODE_FEEDBACK;
	}
	else
	{
		std::cout << "You must select a session mode by using the \"--server\", \"--client=<HOSTNAME>\" or \"--feedback\" command line options." << std::endl << std::endl;
		std::cout << network_options << std::endl;
		return -1;
	}



	if (vm.count("capture-input"))
	{
		if (vm.count("capture-index"))
			captureIndex = vm["capture-index"].as<int>();

		if (vm.count("capture-width"))
			captureInfo.rgbaWidth = captureInfo.zWidth = vm["capture-width"].as<int>();

		if (vm.count("capture-height"))
			captureInfo.rgbaHeight = captureInfo.rgbaHeight = vm["capture-height"].as<int>();

		if (vm.count("capture-fps"))
			captureInfo.rgbFPS = captureInfo.zFPS = vm["capture-fps"].as<float>();

		if (vm["capture-input"].as<std::string>().compare("openni2") == 0)
		{
			capture = holo::capture::HoloCaptureGenerator::fromOpenNI(captureInfo.rgbaWidth, captureInfo.rgbaHeight, captureInfo.rgbFPS, captureInfo.zWidth, captureInfo.zHeight, captureInfo.zFPS);
		}
		else if (!vm["capture-input"].as<std::string>().empty())
		{
			boost::filesystem::path filePath = boost::filesystem::path(vm["capture-input"].as<std::vector<std::string>>()[0]);
			if (boost::filesystem::exists(filePath))
			{
				if (".oni" == boost::filesystem::extension(filePath))
				{
					capture = holo::capture::HoloCaptureGenerator::fromOpenNI(filePath.string());
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
	else if (vm.count("feedback"))
	{
		std::cout << "For feedback mode you must select an capture input device by using the --capture-input command line option." << std::endl << std::endl;
		std::cout << capture_options << std::endl;
		return -1;
	}

	if (vm.count("codec"))
	{
		if (vm["codec"].as<std::string>().compare("octree") == 0)
		{
			holo::codec::HoloCodecOctreeEncodeArgs args;
			args.showStatistics = HOLO_CODEC_OCTREE_DEFAULT_SHOW_STATISTICS;
			args.pointResolution = HOLO_CODEC_OCTREE_DEFAULT_POINT_RESOLUTION;
			args.octreeResolution = HOLO_CODEC_OCTREE_DEFAULT_OCTREE_RESOLUTION;
			args.doVoxelGridDownsampling = HOLO_CODEC_OCTREE_DEFAULT_DO_VOXEL_GRID_DOWNSAMPLING;
			args.frameRate = HOLO_CODEC_OCTREE_DEFAULT_FRAMERATE;
			args.doEncodeColorInfo = HOLO_CODEC_OCTREE_DEFAULT_DO_ENCODE_COLOR_INFO;
			args.colorBitResolution = HOLO_CODEC_OCTREE_DEFAULT_COLOR_BIT_RESOLUTION;

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
					args.showStatistics = atoi(octreeSettings[0].c_str());
					args.pointResolution = atof(octreeSettings[1].c_str());
					args.octreeResolution = atof(octreeSettings[2].c_str());
					args.doVoxelGridDownsampling = atoi(octreeSettings[3].c_str());
					args.frameRate = atoi(octreeSettings[4].c_str());
					args.doEncodeColorInfo = atoi(octreeSettings[5].c_str());
					args.colorBitResolution = atoi(octreeSettings[6].c_str());
				}
				else
				{
					std::cout << "The octree compression settings entered are invalid. Please check the number of arguments and separate values by a comma." << std::endl << std::endl;
					std::cout << codec_options << std::endl;
					return -1;
				}
			}

			if (sessionMode == HOLO_SESSION_MODE_SERVER || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
				codecServer = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression(args);
			if (sessionMode == HOLO_SESSION_MODE_CLIENT || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
			    codecClient = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression(args);
		}
		else if (vm["codec"].as<std::string>().compare("passthrough-cloud") == 0)
		{
			if (sessionMode == HOLO_SESSION_MODE_SERVER || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
				codecServer = holo::codec::HoloCodecGenerator::fromPCLPassthrough();
			if (sessionMode == HOLO_SESSION_MODE_CLIENT || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
				codecClient = holo::codec::HoloCodecGenerator::fromPCLPassthrough();

		}
		else if (vm["codec"].as<std::string>().compare("h264") == 0)
		{
			holo::codec::HoloCodecH264Args args = { 0 };
			args.bitRate = HOLO_CODEC_H264_DEFAULT_BITRATE;
			args.width = captureInfo.zWidth;
			args.height = captureInfo.zHeight;
			args.gopSize = HOLO_CODEC_H264_DEFAULT_GOPSIZE;
			args.maxBFrames = HOLO_CODEC_H264_DEFAULT_MAXBFRAMES;
			args.timeBase = AVRational{ HOLO_CODEC_H264_DEFAULT_TIMEBASE_NUM, captureInfo.zFPS };
			args.pixelFormat = HOLO_CODEC_H264_DEFAULT_PIXELFMT;
			args.zCompressionLevel = HOLO_CODEC_H264_DEFAULT_ZCOMPRESSIONLEVEL;

			if (vm.count("h264-settings"))
			{
				auto h264SettingsString = vm["h264-settings"].as<std::string>();
				std::vector<std::string> h264Settings;
				boost::char_separator<char> sep(", ");
				boost::tokenizer<boost::char_separator<char>> tokens(h264SettingsString, sep);
				for (const auto& t : tokens) {
					h264Settings.push_back(t);
				}

				if (h264Settings.size() == 3)
				{
					args.bitRate = atoi(h264Settings[0].c_str());
					args.gopSize = atoi(h264Settings[1].c_str());
					args.maxBFrames = atoi(h264Settings[2].c_str());
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
				args.zCompressionLevel = vm["h264-z-level"].as<int>();
			}

			if (sessionMode == HOLO_SESSION_MODE_SERVER || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
				codecServerRGBAZ = holo::codec::HoloCodecGenerator::fromH264(args);
			if (sessionMode == HOLO_SESSION_MODE_CLIENT || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
				codecClientRGBAZ = holo::codec::HoloCodecGenerator::fromH264(args);
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

	if (vm.count("render-output"))
	{
		if (vm["render-output"].as<std::string>().compare("visualizer") == 0)
		{
			int voxelSize = HOLO_RENDER_DEFAULT_VOXEL_SIZE;

			if (vm.count("visualizer-settings"))
			{
				auto visSettingsString = vm["visualizer-settings"].as<std::string>();
				std::vector<std::string> visSettings;
				boost::char_separator<char> sep(", ");
				boost::tokenizer<boost::char_separator<char>> tokens(visSettingsString, sep);
				for (const auto& t : tokens) {
					visSettings.push_back(t);
				}

				if (visSettings.size() == 1)
				{
					voxelSize = atoi(visSettings[0].c_str());
				}
				else
				{
					std::cout << "The visualizer settings entered are invalid. Please check the number of arguments and separate values by a comma." << std::endl << std::endl;
					std::cout << render_options << std::endl;
					return -1;
				}
			}

			renderer = holo::render::HoloRenderGenerator::fromPCLVisualizer(voxelSize, captureInfo.zWidth, captureInfo.zHeight);
		}
		else
		{
			std::cout << "Invalid renderer selection. Please use a valid renderer such as --reneder-output=\"visualizer\"." << std::endl << std::endl;
			std::cout << render_options << std::endl;
			return -1;
		}
	}
	else if (vm.count("feedback"))
	{
		std::cout << "For feedback mode you must select a renderer by using the --render-output command line option." << std::endl << std::endl;
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
	case HOLO_SESSION_MODE_SERVER:
		LOG4CXX_INFO(logger_main, "Holosuite starting in server mode...");
		break;
	case HOLO_SESSION_MODE_CLIENT:
		LOG4CXX_INFO(logger_main, "Holosuite starting in client mode...");
		break;
	case HOLO_SESSION_MODE_FEEDBACK:
		LOG4CXX_INFO(logger_main, "Holosuite starting in feedback mode...");
		break;
	default:
		break;
	}

	bool success = false;
	
	if (capture)
	{
		if (!capture->init(captureIndex))
		{
			LOG4CXX_FATAL(logger_main, "Could not open the capture input.  Exiting holosuite...");
			return -1;
		}

		captureInfo = capture->getCaptureInfo();
	}

	switch (sessionMode)
	{
	case HOLO_SESSION_MODE_SERVER:
		if(codecServer)
			success = codecServer->init(holo::codec::CODEC_MODE_BOTH);
		else if (codecServerRGBAZ)
			success = codecServerRGBAZ->init(holo::codec::CODEC_MODE_BOTH);
		break;
	case HOLO_SESSION_MODE_CLIENT:
		if (codecClient)
			success = codecClient->init(holo::codec::CODEC_MODE_BOTH);
		else if (codecClientRGBAZ)
			success = codecClientRGBAZ->init(holo::codec::CODEC_MODE_BOTH);
		break;
	case HOLO_SESSION_MODE_FEEDBACK:
		if (codecServer)
			success = codecServer->init(holo::codec::CODEC_MODE_BOTH) && codecClient->init(holo::codec::CODEC_MODE_BOTH);
		else if (codecServerRGBAZ)
			success = codecServerRGBAZ->init(holo::codec::CODEC_MODE_ENCODER) && codecClientRGBAZ->init(holo::codec::CODEC_MODE_DECODER);
		break;
	default:
		break;
	}
	
	if (!success)
	{
		logger_main->fatal("Could not initialize the codec. Exiting holosuite...");
		return -1;
	}

	if (renderer)
		if (!renderer->init())
		{
			logger_main->fatal("Could not initialize the renderer. Exiting holosuite...");
			return -1;
		}

		strcpy((char*)localInfo.clientName, sessionName.c_str());
		localInfo.rgbazWidth = captureInfo.zWidth;
		localInfo.rgbazHeight = captureInfo.zHeight;
		localInfo.captureFPS = captureInfo.zFPS;
		localInfo.captureHOV = captureInfo.zHOV;
		localInfo.captureVOV = captureInfo.zVOV;

	switch (sessionMode)
	{
	case HOLO_SESSION_MODE_SERVER:
	{
		infoFromClient = server->listenAndWait(HOLO_NET_DEFAULT_PORT, localInfo);
		if (codecServer)
			serverSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(capture), std::move(codecServer), server, infoFromClient, std::move(renderer)));
		else
			serverSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(capture), std::move(codecServerRGBAZ), server, infoFromClient, std::move(renderer)));
		serverSession->start();
	}
		break;
	case HOLO_SESSION_MODE_CLIENT:
	{
		 infoFromServer = client->connect(remoteAddress, HOLO_NET_DEFAULT_PORT, localInfo);
		if (codecClient)
			clientSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(capture), std::move(codecClient), client, infoFromServer, std::move(renderer)));
		else
			clientSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(capture), std::move(codecClientRGBAZ), client, infoFromServer, std::move(renderer)));
		
		clientSession->start();
	}
		break;
	case HOLO_SESSION_MODE_FEEDBACK:
	{
		auto serverFunc = std::bind<holo::net::HoloNetProtocolHandshake>(&holo::net::HoloNetServer::listenAndWait, server, HOLO_NET_DEFAULT_PORT, std::placeholders::_1);
		auto servhandle = std::future<holo::net::HoloNetProtocolHandshake>(std::async(std::launch::async,serverFunc, localInfo));
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		infoFromServer = client->connect(remoteAddress, HOLO_NET_DEFAULT_PORT, localInfo);

		infoFromClient = servhandle.get();

		if (codecClient)
			clientSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(nullptr), std::move(codecClient), client, infoFromServer, std::move(renderer)));
		else
			clientSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(nullptr), std::move(codecClientRGBAZ), client, infoFromServer, std::move(renderer)));

		if (codecServer)
			serverSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(capture), std::move(codecServer), server, infoFromClient, std::move(nullptr)));
		else serverSession = std::unique_ptr<holo::HoloSession>(new holo::HoloSession(std::move(capture), std::move(codecServerRGBAZ), server, infoFromClient, std::move(nullptr)));
				
		clientSession->start();
		serverSession->start();

	}
		break;
	default:
		break;
	}

	while (true)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	}


	return 0;
}
