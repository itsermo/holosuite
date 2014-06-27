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

	std::string sessionName;
	std::string remoteAddress;
	HOLO_SESSION_MODE sessionMode;
	holo::net::HoloNetProtocolHandshake localInfo;
	holo::net::HoloNetProtocolHandshake infoFromClient;
	holo::net::HoloNetProtocolHandshake infoFromServer;
	holo::capture::HoloCaptureInfo captureInfo = {0};

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
		("verbosity,v", boost::program_options::value<int>()->composing(), "level of detail for console output. valid values are [0-4] from least to most verbose, default is 1")
		("help,h", "produce help message")
		;

	// Network connection options
	boost::program_options::options_description network_options("Network session options");
	network_options.add_options()
		("name", boost::program_options::value<std::string>()->composing(), "friendly name of the server or client session (optional)")
		("server", "start holosuite in server mode, listen for connections")
		("client", boost::program_options::value<std::string>()->composing(), "start holosuite in client mode. (e.g. client=192.168.0.3)")
		("feedback", "capture input is routed to local renderer" )
		;
    
	// Capture input options
	boost::program_options::options_description capture_options("Capture input source");
	capture_options.add_options()
		("capture-input,i",
			 boost::program_options::value< std::vector<std::string> >()->composing(), 
			 "selects the capture method. valid setting is [openni2]")
		;

	// Codec options
	boost::program_options::options_description codec_options("Codec");
	codec_options.add_options()
		("codec,c",
		boost::program_options::value<std::vector<std::string> >()->composing(),
		"valid settings are [octree,h264]")
		;

	// Render options
	boost::program_options::options_description render_options("Renderer");
	render_options.add_options()
		("render-output,o",
		boost::program_options::value<std::vector<std::string> >()->composing(),
		"valid setting is [visualizer]")
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
		std::cout << "Holosuite is a digital holographic video telepresence software suite." << std::endl;
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
			std::cout << "Invalid client address.  Please use --client=<HOSTNAME> format to connect to a remote holosuite server." << std::endl << std::endl;
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
		if (vm["capture-input"].as<std::vector<std::string>>()[0].compare("openni2") == 0)
		{
			capture = holo::capture::HoloCaptureGenerator::fromOpenNI();
		}
		else if (!vm["capture-input"].as<std::vector<std::string>>()[0].empty())
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
		if (vm["codec"].as<std::vector<std::string>>()[0].compare("octree") == 0)
		{
			if (sessionMode == HOLO_SESSION_MODE_SERVER || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
				codecServer = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression();
			if (sessionMode == HOLO_SESSION_MODE_CLIENT || sessionMode == HOLO_SESSION_MODE_FEEDBACK)
			    codecClient = holo::codec::HoloCodecGenerator::fromPCLOctreeCompression();
		}
		else if (vm["codec"].as<std::vector<std::string>>()[0].compare("passthrough-cloud") == 0)
		{
			codecServer = holo::codec::HoloCodecGenerator::fromPCLPassthrough();
			codecClient = holo::codec::HoloCodecGenerator::fromPCLPassthrough();
		}
		else if (vm["codec"].as<std::vector<std::string>>()[0].compare("h264") == 0)
		{
			codecClientRGBAZ = holo::codec::HoloCodecGenerator::fromH264();
			codecServerRGBAZ = holo::codec::HoloCodecGenerator::fromH264();
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
		if (vm["render-output"].as<std::vector<std::string>>()[0].compare("visualizer") == 0)
		{
			renderer = holo::render::HoloRenderGenerator::fromPCLVisualizer();
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
		if (!capture->init(0))
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
			success = codecServer->init(holo::codec::CODEC_TYPE_BOTH);
		else if (codecServerRGBAZ)
			success = codecServerRGBAZ->init(holo::codec::CODEC_TYPE_BOTH);
		break;
	case HOLO_SESSION_MODE_CLIENT:
		if (codecClient)
			success = codecClient->init(holo::codec::CODEC_TYPE_BOTH);
		else if (codecClientRGBAZ)
			success = codecClientRGBAZ->init(holo::codec::CODEC_TYPE_BOTH);
		break;
	case HOLO_SESSION_MODE_FEEDBACK:
		if (codecServer)
			success = codecServer->init(holo::codec::CODEC_TYPE_BOTH) && codecClient->init(holo::codec::CODEC_TYPE_BOTH);
		else if (codecServerRGBAZ)
			success = codecServerRGBAZ->init(holo::codec::CODEC_TYPE_ENCODER) && codecClientRGBAZ->init(holo::codec::CODEC_TYPE_DECODER);
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
