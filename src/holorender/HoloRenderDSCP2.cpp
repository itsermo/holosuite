#ifdef ENABLE_HOLO_DSCP2

#include "HoloRenderDSCP2.hpp"
#include "../holoutils/HoloUtils.hpp"

#include <boost/filesystem.hpp>
#include <future>

#if defined(__linux) || defined(__unix) || defined(__posix)
#include <X11/Xlib.h>
#endif

using namespace holo;
using namespace holo::render;

HoloRenderDSCP2::HoloRenderDSCP2(int numHeads) : IHoloRender(),
	haveNewCloud_(false),
	numHeads_(numHeads),
	masterHologramGain_(HOLO_RENDER_DSCP2_MASTER_HOLOGRAM_GAIN),
	viewEnableBitmask_(HOLO_RENDER_DSCP2_VIEW_ENABLE_BITMASK),
	zeroDepth_(HOLO_RENDER_DSCP2_ZERO_DEPTH),
	zeroModulation_(HOLO_RENDER_DSCP2_ZERO_MODULATION),
	fakeZ_(HOLO_RENDER_DSCP2_FAKE_Z),
	fakeModulation_(HOLO_RENDER_DSCP2_FAKE_MODULATION),
	hologramOutputDebugSwitch_(0),
	enableDrawDepth_(0), rotateCounter_(0),
	frameNumber_(0), currentTime_(0), timeBase_(0),
	displayModeWidth_(HOLO_RENDER_DSCP2_DISPLAY_MODE_WIDTH),
	displayModeHeight_(HOLO_RENDER_DSCP2_DISPLAY_MODE_HEIGHT),
	hologramPlaneWidthMM_(HOLO_RENDER_DSCP2_VIEW_RENDER_HOLOGRAM_PLANE_WIDTH_MM),
	hologramPlaneHeightMM_(HOLO_RENDER_DSCP2_VIEW_RENDER_HOLOGRAM_PLANE_HEIGHT_MM),
	nearPlane_(HOLO_RENDER_DSCP2_PLANE_NEAR),
	farPlane_(HOLO_RENDER_DSCP2_PLANE_FAR),
	numX_(HOLO_RENDER_DSCP2_VIEW_RENDER_RES_HORIZ),
	numY_(HOLO_RENDER_DSCP2_VIEW_RENDER_RES_VERT),
	tileX_(HOLO_RENDER_DSCP2_VIEW_RENDER_TILE_X),
	tileY_(HOLO_RENDER_DSCP2_VIEW_RENDER_TILE_Y),
	numViewsPerPixel_(HOLO_RENDER_DSCP2_VIEW_RENDER_NUM_VIEWS_PER_PIXEL),
	mag_(HOLO_RENDER_DSCP2_VIEW_RENDER_MAG),
	fieldOfView_(HOLO_RENDER_DSCP2_VIEW_RENDER_FOV),
	lightLocationX_(HOLO_RENDER_DSCP2_LIGHT_LOCATION_X),
	lightLocationY_(HOLO_RENDER_DSCP2_LIGHT_LOCATION_Y),
	lightLocationZ_(HOLO_RENDER_DSCP2_LIGHT_LOCATION_Z),
	translateX_(0.0f),
	translateY_(0.0f),
	translateZ_(0.0f),
	rot_(0.0f),
	rotX_(0.0f),
	texNum_(0),
	vertexProgramName_(HOLO_RENDER_DSCP2_CG_VERTEX_PROGRAM_NAME),
	vertexProgramFileName_(HOLO_RENDER_DSCP2_CG_VERTEX_PROGRAM_FILENAME),
	fragmentProgramName_(HOLO_RENDER_DSCP2_CG_FRAGMENT_PROGRAM_NAME),
	fragmentProgramFileName_(HOLO_RENDER_DSCP2_CG_FRAGMENT_PROGRAM_FILENAME),
	normalMapLightingVertexProgramName_(HOLO_RENDER_DSCP2_CG_NORMAL_MAPLIGHT_VERTEX_PROGRAM_NAME),
	normalMapLightingVertexProgramFileName_(HOLO_RENDER_DSCP2_CG_NORMAL_MAPLIGHT_VERTEX_PROGRAM_FILENAME),
	normalMapLightingFragmentProgramName_(HOLO_RENDER_DSCP2_CG_NORMAL_MAPLIGHT_FRAGMENT_PROGRAM_NAME),
	normalMapLightingFragmentProgramFileName_(HOLO_RENDER_DSCP2_CG_NORMAL_MAPLIGHT_FRAGMENT_PROGRAM_FILENAME),
	normalMapLightingCgContext_(nullptr),
	normalMapLightingCgVertexProgram_(nullptr),
	normalMapLightingCgFragmentProgram_(nullptr),
	normalMapLightingCgVertexProfile_(CG_PROFILE_UNKNOWN),
	normalMapLightingCgFragmentProfile_(CG_PROFILE_UNKNOWN),
	cgVertexParamModelViewProj_(nullptr),
	cgFragmentParamGlobalAmbient_(nullptr),
	cgFragmentParamLightColor_(nullptr),
	cgFragmentParamLightPosition_(nullptr),
	cgFragmentParamEyePosition_(nullptr),
	cgFragmentParamKe_(nullptr),
	cgFragmentParamKa_(nullptr),
	cgFragmentParamKd_(nullptr),
	cgFragmentParamKs_(nullptr),
	cgFragmentParamShininess_(nullptr),
	cgFragmentParamDrawdepth_(nullptr),
	cgFragmentParamHeadnum_(nullptr),
	cgFragmentParamHogelYes_(nullptr),
	cgFragmentParamHologramGain_(nullptr),
	cgFragmentParamHologramDebugSwitch_(nullptr),
	cgVertexParamTextureMatrix_(nullptr),
	cgVertexParamDepthMatrix_(nullptr),
	cgVertexParamDrawdepth_(nullptr),
	cgVertexProfile_(CG_PROFILE_UNKNOWN),
	cgFragmentProfile_(CG_PROFILE_UNKNOWN),
	localFramebufferStore_(nullptr),
	firstInit_(true),
	isInit_(false)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.dscp2");

	LOG4CXX_DEBUG(logger_, "Instantiating HoloRenderDSCP2 object with default values...");

	viewTexWidth_ = numX_ * tileX_;
	viewTexHeight_ = numY_ * tileY_ * 2; //tiley views of numy pixels, X 2 (depth, luminance) was 256;
	numViews_ = tileX_ * tileY_ * numViewsPerPixel_;

	gCurrentInstance = this;

	lightAmbient_[0] = 0.5f;
	lightAmbient_[1] = 0.5f;
	lightAmbient_[2] = 0.5f;
	lightAmbient_[3] = 1.0f;

	lightDiffuse_[0] = 1.0f;
	lightDiffuse_[1] = 1.0f;
	lightDiffuse_[2] = 1.0f;
	lightDiffuse_[3] = 1.0f;

	// set light position
	lightPosition_[0] = 0.0f;
	lightPosition_[1] = 0.0f;
	lightPosition_[2] = -2.0f;
	lightPosition_[3] = 1.0f;

	// set light color to white { 0.95f, 0.95f, 0.95f }
	lightColor_[0] = 0.95f;
	lightColor_[1] = 0.95f;
	lightColor_[2] = 0.95f;

	// set to dim color { 0.1f, 0.1f, 0.1f }
	globalAmbient_[0] = 0.1f;
	globalAmbient_[1] = 0.1f;
	globalAmbient_[2] = 0.1f;

	memset(projectionMatrix1_, 0, sizeof(GLfloat)* 16);
	memset(projectionMatrix2_, 0, sizeof(GLfloat)* 16);

	localFramebufferStore_ = new GLubyte[viewTexWidth_ * viewTexHeight_ * 4];

	// Get the screen resolution from the desktop (Windows or X11)
#ifdef WIN32
	RECT desktop;
	// Get a handle to the desktop window
	const HWND hDesktop = GetDesktopWindow();
	// Get the size of screen to the variable desktop
	GetWindowRect(hDesktop, &desktop);
	// The top left corner will have coordinates (0,0)
	// and the bottom right corner will have coordinates
	// (horizontal, vertical)
	displayModeWidth_ = desktop.right;
	displayModeHeight_ = desktop.bottom;
#elif defined(__linux) || defined(__unix) || defined(__posix)
	Display *displayName;
	int depth, screen, connection;

	/*Opening display and setting defaults*/
	displayName = XOpenDisplay(":0.0");
	screen = DefaultScreen(displayName);

	displayModeWidth_ = DisplayWidth(displayName, screen);
	displayModeHeight_ = DisplayHeight(displayName, screen);
#endif

	LOG4CXX_INFO(logger_, "Window environment display mode resolution: " << displayModeWidth_ << "x" << displayModeHeight_);


	LOG4CXX_DEBUG(logger_, "Done instantiating HoloRenderDSCP2 object");
}

HoloRenderDSCP2::HoloRenderDSCP2() : HoloRenderDSCP2(1)
{

}

HoloRenderDSCP2::~HoloRenderDSCP2()
{
	LOG4CXX_DEBUG(logger_, "Destroying HoloRenderDSCP2 object...");

	if (localFramebufferStore_)
	{
		delete[] localFramebufferStore_;
		localFramebufferStore_ = nullptr;
	}

	LOG4CXX_DEBUG(logger_, "Done destroying HoloRenderDSCP2 object");
}

bool HoloRenderDSCP2::init()
{
	LOG4CXX_INFO(logger_, "Initializing DSCP2 render algorithm...");

	cloud_ = HoloCloudPtr(new HoloCloud);

	glutInitThread_ = std::thread(&HoloRenderDSCP2::glutInitLoop, this);
	
	std::unique_lock<std::mutex> lg(hasInitMutex_);
	hasInitCV_.wait(lg);

	return isInit_;
}

void HoloRenderDSCP2::glutInitLoop()
{
	GLenum glError = GL_NO_ERROR;
	CGerror cgError = CG_NO_ERROR;
	char fakeParam[] = "fake";
	char *fakeargv[] = { fakeParam, NULL };
	int fakeargc = 1;

	//glCheckErrors();

	// Initialize GLUT window and callbacks
	glutInit(&fakeargc, fakeargv);
	glutInitWindowSize(displayModeWidth_, displayModeHeight_);
	glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);
	
	for (int i = 0; i < numHeads_; i++)
	{
		glutCreateWindow(HOLO_RENDER_DSCP2_CG_PROGRAM_NAME);


		glCheckErrors();

		glutDisplayFunc(this->glutDisplay);
		glutKeyboardFunc(this->glutKeyboard);
		glutIdleFunc(this->glutIdle);
		atexit(this->glutCleanup);
	}

	// GLUT settings
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Black Background
	glShadeModel(GL_SMOOTH); // Enable Smooth Shading

	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient_); // Setup The Ambient Light
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse_); // Setup The Diffuse Light
	glLightfv(GL_LIGHT0, GL_POSITION, lightPosition_); // Position The Light
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST); // Enables Depth Testing
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	// Creates a texture ID for our panoramagram generation
	glGenTextures(1, &textureID_);

	//double q = tan(0.1);
	//holo::utils::BuildShearOrthographicMatrix2(-0.75f * mag_, -0.75f * mag_, -0.375f * mag_, 0.375f * mag_, 0.450f * mag_, 0.750f * mag_, q / mag_, projectionMatrix1_);

	// CG program intialization
	cgGLSetDebugMode(CG_FALSE);

	normalMapLightingCgContext_ = cgCreateContext();
	checkForCgError("Creating normal map lighting context...");

	// Panoramagram generation vertex program
	normalMapLightingCgVertexProfile_ = cgGLGetLatestProfile(CG_GL_VERTEX);
	cgGLSetOptimalOptions(normalMapLightingCgVertexProfile_);
	checkForCgError("Selecting vertex profile...");
	normalMapLightingCgVertexProgram_ = cgCreateProgramFromFile(normalMapLightingCgContext_, CG_SOURCE, normalMapLightingVertexProgramFileName_, normalMapLightingCgVertexProfile_,normalMapLightingVertexProgramName_,NULL);
	checkForCgError("Creating vertex program from file");
	cgGLLoadProgram(normalMapLightingCgVertexProgram_);
	checkForCgError("Loading vertex program");
	LOG4CXX_INFO(logger_, "Loaded panoramagram vertex program file: " << normalMapLightingVertexProgramFileName_);

	cgVertexParamModelViewProj_ = cgGetNamedParameter(normalMapLightingCgVertexProgram_, "modelViewProj");
	checkForCgError("could not get modelViewProj vertex parameter ln 1707");
	//cgVertexParamTextureMatrix_ = cgGetNamedParameter(normalMapLightingCgVertexProgram_, "textureMatrix");
	//checkForCgError("could not get textureMatrix vertex parameter ln 1707");
	//cgVertexParamDepthMatrix_ = cgGetNamedParameter(normalMapLightingCgVertexProgram_, "depthMatrix");
	//checkForCgError("could not get depthMatrix vertex parameter ln 1707");
	//cgVertexParamDrawdepth_ = cgGetNamedParameter(normalMapLightingCgVertexProgram_, "drawDepth");
	//checkForCgError("could not get drawDepth vertex parameter ln 1707");

	// Panoramagram generation fragment program
	normalMapLightingCgFragmentProfile_ = cgGLGetLatestProfile(CG_GL_FRAGMENT);
	cgGLSetOptimalOptions(normalMapLightingCgFragmentProfile_);
	checkForCgError("Selecting fragment profile...");
	normalMapLightingCgFragmentProgram_ = cgCreateProgramFromFile(normalMapLightingCgContext_, CG_SOURCE, normalMapLightingFragmentProgramFileName_, normalMapLightingCgFragmentProfile_, normalMapLightingFragmentProgramName_, NULL); 
	checkForCgError("Creating fragment program");
	cgGLLoadProgram(normalMapLightingCgFragmentProgram_);
	checkForCgError("loading fragment program");
	LOG4CXX_INFO(logger_, "Loaded panoramagram fragment program file: " << normalMapLightingFragmentProgramFileName_);

	cgFragmentParamGlobalAmbient_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "globalAmbient");
	cgFragmentParamLightColor_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "lightColor");
	cgFragmentParamLightPosition_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "lightPosition");
	cgFragmentParamEyePosition_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "eyePosition");
	cgFragmentParamKe_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "Ke");
	cgFragmentParamKa_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "Ka");
	cgFragmentParamKd_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "Kd");
	cgFragmentParamKs_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "Ks");
	cgFragmentParamShininess_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "shininess");
	cgFragmentParamDrawdepth_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "drawdepth");
	cgFragmentParamHeadnum_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "headnum");

	cgFragmentParamDecal0_ = cgGetNamedParameter(normalMapLightingCgFragmentProgram_, "decal");
	checkForCgError("Getting fragment program decal parameter");

	cgGLSetTextureParameter(cgFragmentParamDecal0_, textureID_);
	checkForCgError("Setting decal texture");

	// Set light source color parameters once. 
	cgSetParameter3fv(cgFragmentParamGlobalAmbient_, globalAmbient_);
	checkForCgError("Setting fragment global ambient lighting");
	cgSetParameter3fv(cgFragmentParamLightColor_, lightColor_);
	checkForCgError("Setting fragment program light color");

	// Set up head number for rendering/loading with skipped lines
	cgSetParameter1i(cgFragmentParamHeadnum_, numHeads_);
	checkForCgError("Setting head number parameter");

	// Set up view texture (holds all view images. TODO: convert to 2 3d textures
	glBindTexture(GL_TEXTURE_2D, textureID_);

	// Set texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

	// Specify a 2-dimensional texture image and uploads it to video memory, make it ready to use in shaders
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, viewTexWidth_, viewTexHeight_, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);

	boost::filesystem::path workingDir(boost::filesystem::current_path());
	LOG4CXX_INFO(logger_, "Accessing Cg files from working directory: " << workingDir.string());

	// Fringe computation vertex program
	cgVertexProfile_ = cgGLGetLatestProfile(CG_GL_VERTEX);
	cgGLSetOptimalOptions(cgVertexProfile_);
	checkForCgError("Selecting vertex profile");
	cgVertexProgram_ = cgCreateProgramFromFile(normalMapLightingCgContext_,	CG_SOURCE,	vertexProgramFileName_, cgVertexProfile_, vertexProgramName_, NULL);
	checkForCgError2("Creating fringe vertex program from file");
	cgGLLoadProgram(cgVertexProgram_);
	checkForCgError2("Loading vertex program");
	LOG4CXX_INFO(logger_, "Loaded fringe computation vertex program file: " << vertexProgramFileName_);

	// Fringe computation fragment program
	cgFragmentProfile_ = cgGLGetLatestProfile(CG_GL_FRAGMENT);
	cgGLSetOptimalOptions(cgFragmentProfile_);
	checkForCgError2("Selecting fragment profile");
	cgFragmentProgram_ = cgCreateProgramFromFile(normalMapLightingCgContext_, CG_SOURCE, fragmentProgramFileName_, cgFragmentProfile_, fragmentProgramName_, NULL);
	checkForCgError2("creating fragment program from file");
	cgGLLoadProgram(cgFragmentProgram_);
	checkForCgError2("loading fragment program");
	LOG4CXX_INFO(logger_, "Loaded fringe computation fragment program file: " << vertexProgramFileName_);

	cgFragmentParamHogelYes_ = cgGetNamedParameter(cgFragmentProgram_, "hogelYes"); 
	cgSetParameter1f(cgFragmentParamHogelYes_, 0.0f);
	cgFragmentParamHologramGain_ = cgGetNamedParameter(cgFragmentProgram_, "hologramGain");
	cgSetParameter1f(cgFragmentParamHologramGain_, masterHologramGain_);
	cgFragmentParamHologramDebugSwitch_ = cgGetNamedParameter(cgFragmentProgram_, "hologramDebugSwitch");
	cgSetParameter1f(cgFragmentParamHologramDebugSwitch_, hologramOutputDebugSwitch_);
	cgFragmentParamHeadnum_ = cgGetNamedParameter(cgFragmentProgram_, "headnum");
	cgSetParameter1f(cgFragmentParamHeadnum_, numHeads_);
	cgFragmentParamDecal1_ = cgGetNamedParameter(cgFragmentProgram_,"decal0");
	checkForCgError2("getting decal parameter");

	cgGLSetTextureParameter(cgFragmentParamDecal1_, textureID_);
	checkForCgError2("setting decal 3D texture0");

	glClearColor(0, 0, 0, 0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, static_cast<float>(displayModeWidth_) / static_cast<float>(displayModeHeight_), 0.01f, 3.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_LIGHTING);

	glCheckErrors();

	glutMainLoop();
}

void HoloRenderDSCP2::glCheckErrors()
{
	GLenum error;

	while ((error = glGetError()) != GL_NO_ERROR)
	{
		LOG4CXX_WARN(logger_, "OpenGL error: " << gluErrorString(error));
	}
}

void HoloRenderDSCP2::idle()
{
	// refresh point cloud data
	if (haveNewCloud_.load())
		glutPostRedisplay();

#ifdef TRACE_LOG_ENABLED

	float fps;
	if (rotateCounter_ == 1)
	{
		rot_ = rot_ - 5;
		rot_ = fmod(rot_, 360);
	}
	//tz=tz-.5;
	//  glutPostRedisplay();

	frameNumber_++;
	currentTime_ = glutGet(GLUT_ELAPSED_TIME);
	//printf("idle\n");
	if (currentTime_ - timeBase_ > 1000)
	{
		const int len = 1024;
		char msg[len];
		//	printf("here\n");
		fps = frameNumber_ * 1000.0 / (currentTime_ - timeBase_);
		timeBase_ = currentTime_;
		frameNumber_ = 0;
		//sprintf(msg, "Wafel %d fps: %f\n", headNumber_, fps);
		//printf("%s", msg);

		LOG4CXX_TRACE(logger_, "DSCP2 rendering on head " << headNumber_, << " @ " << fps << " fps");
//		fflush(stdout);
	}

#endif
}

void HoloRenderDSCP2::keyboard(unsigned char c, int x, int y)
{
	//printf("keyboard \n");
	switch (c)
	{

	case 'z': //invert set of disabled views
		viewEnableBitmask_ = ~viewEnableBitmask_;
		break;
	case 'Z': //enable all views
		viewEnableBitmask_ = -1;
		break;
	case 'x': //cycle through debug modes in shader (output intermediate variables)
		hologramOutputDebugSwitch_++;
		hologramOutputDebugSwitch_ = hologramOutputDebugSwitch_ % HOLO_RENDER_DSCP2_NUM_DEBUG_SWITCHES;
		cgSetParameter1f(cgFragmentParamHologramDebugSwitch_, hologramOutputDebugSwitch_);
		break;
	case 'X':
		hologramOutputDebugSwitch_--;
		hologramOutputDebugSwitch_ = hologramOutputDebugSwitch_ % HOLO_RENDER_DSCP2_NUM_DEBUG_SWITCHES;
		cgSetParameter1f(cgFragmentParamHologramDebugSwitch_, hologramOutputDebugSwitch_);
		break;
	case 'j':
		translateX_ = translateX_ + 5;
		printf("tx %f \n", translateX_);
		break;
	case 'l':
		translateX_ = translateX_ - 5;
		printf("tx %f \n", translateX_);
		break;
	case 'i':
		translateY_ = translateY_ - 1;
		printf("ty %f \n", translateY_);
		break;
	case 'k':
		translateY_ = translateY_ + 1;
		printf("ty %f \n", translateY_);
		break;

	case 'w':
		translateZ_ = translateZ_ - 1;
		printf("%f \n", translateZ_ - 675);
		break;
	case 's':
		translateZ_ = translateZ_ + 1;
		printf("%f \n", translateZ_ - 675);
		break;

	case 'f':
		//writeToFile2();
		break;
	case 'F':
		//printf("writing view texture\n");
		//writeViewsToFile();
		break;
	case 'e':
		translateZ_ = translateZ_ - 10;
		printf("%f \n", translateZ_ - 675);
		break;
	case 'd':
		translateZ_ = translateZ_ + 10;
		printf("%f \n", translateZ_ - 675);
		break;
	case 'c':
		translateZ_ = 0;
		printf("%f \n", translateZ_ - 675);
		break;
	case 'r':
		rot_ = rot_ - 5;
		rotateCounter_ = (rotateCounter_ * -1) + 1;

		printf("rotate %i \n", rotateCounter_);
		break;
	case ' ':
		//makeViewtexFromFile();
		break;
	case ']':
		lightLocationZ_ = lightLocationZ_ - 1.0f;
		printf("%f \n", lightLocationZ_);
		break;
	case '[':
		lightLocationZ_ = lightLocationZ_ + 1.0f;
		printf("%f \n", lightLocationZ_);
		break;
	case '=':
		lightLocationY_ = lightLocationY_ - 1.0f;
		printf("%f \n", lightLocationY_);
		break;
	case '-':
		lightLocationY_ = lightLocationY_ + 1.0f;
		printf("%f \n", lightLocationY_);
		break;
	case ';':
		lightLocationX_ = lightLocationX_ - 1.0f;
		printf("%f \n", lightLocationX_);
		break;
	case '/':
		lightLocationX_ = lightLocationX_ + 1.0f;
		printf("%f \n", lightLocationX_);
		break;
	case '1':

		glEnable(GL_LIGHTING);
		//cgSetParameter1f(myCgFragmentParam_hogelYes, 0.);
		//cgUpdateProgramParameters(myCgFragmentProgram2);
		//printf("Wafel");
		break;
	case '2':
		glDisable(GL_LIGHTING);
		//cgSetParameter1f(myCgFragmentParam_hogelYes, 1.);
		//cgUpdateProgramParameters(myCgFragmentProgram2);
		//printf("Hogel");
		break;

	case 27: /* Esc key */
		/* Demonstrate proper deallocation of Cg runtime data structures.
		Not strictly necessary if we are simply going to exit. */
		cgDestroyProgram(normalMapLightingCgVertexProgram_);
		cgDestroyContext(normalMapLightingCgContext_);
		// ShutDown();
		exit(0);
		break;
	}

	int mods = glutGetModifiers();
	if (mods != 0)
	{
		if (c >= '0' && c <= '9') viewEnableBitmask_ ^= 1 << (c - '0' + 10); //toggle view enable bit for numbered view 10-19 (only 16 views used)
	}
	else {
		if (c >= '0' && c <= '9') viewEnableBitmask_ ^= 1 << (c - '0'); //toggle view enable bit for numbered view 0-9

	}

	glutPostRedisplay();
}

void HoloRenderDSCP2::display()
{
	if (firstInit_)
	{
		//cv::namedWindow("TextureView", CV_WINDOW_FREERATIO);
		firstInit_ = false;
		isInit_ = true;
		hasInitCV_.notify_all();
	}
	
	glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#if 1
	/* World-space positions for light and eye. */
	float eyePosition[4] = { 0, 0, 0, 1 };

	const float lightPosition[4] = { .010f * mag_ + lightLocationX_, .020f * mag_ + lightLocationY_, -0.605f * mag_ + lightLocationZ_, 1.0f };
	float xpos, h, v, rgba, scale;
	int i, j;
	float myobject;

	float translateMatrix[16], rotateMatrix[16], rotateMatrix1[16],
		translateNegMatrix[16], rotateTransposeMatrix[16],
		rotateTransposeMatrix1[16], scaleMatrix[16], scaleNegMatrix[16],
		modelMatrix_sphere[16], invModelMatrix_sphere[16],
		modelMatrix_cone[16], invModelMatrix_cone[16],
		objSpaceLightPosition_sphere[16], objSpaceLightPosition_cone[16],
		objSpaceEyePosition_sphere[16];

		glPushAttrib(GL_VIEWPORT_BIT | GL_COLOR_BUFFER_BIT);
		glEnable(GL_TEXTURE_2D);

		//glBindTexture(GL_TEXTURE_2D, textureID_);

		cgGLBindProgram(normalMapLightingCgVertexProgram_);
		checkForCgError("ln943 binding vertex program lighting");
		cgGLEnableProfile(normalMapLightingCgVertexProfile_);
		checkForCgError("ln945 enabling vertex profile lighting");

		cgGLBindProgram(normalMapLightingCgFragmentProgram_);
		checkForCgError("ln949 binding vertex program lighting");
		cgGLEnableProfile(normalMapLightingCgFragmentProfile_);
		checkForCgError("ln951 enabling vertex profile lighting");
		/*for sphere find model and invModelMatrix */

		//TODO: recompute these only on change:
		holo::utils::MakeRotateMatrix(rot_, 0, 1, 0, rotateMatrix);
		holo::utils::MakeRotateMatrix(-rot_, 0, 1, 0, rotateTransposeMatrix);

		holo::utils::MakeRotateMatrix(180 + rotX_, 1, 0, 0, rotateMatrix1);
		holo::utils::MakeRotateMatrix(-180 - rotX_, 1, 0, 0, rotateTransposeMatrix1);

		holo::utils::MultMatrix(rotateMatrix, rotateMatrix1, rotateMatrix);
		holo::utils::MultMatrix(rotateTransposeMatrix, rotateTransposeMatrix1, rotateTransposeMatrix);

		//z is -600 + tz (z shift tz is centered halfway between near & far planes)
		holo::utils::MakeTranslateMatrix(translateX_, translateY_, -(farPlane_ + nearPlane_) * 0.5 * mag_ + translateZ_ * mag_,	translateMatrix);
		holo::utils::MakeTranslateMatrix(-translateX_, -translateY_, (farPlane_ + nearPlane_) * 0.5 * mag_ - translateZ_ * mag_, translateNegMatrix);

		scale = 2;
		holo::utils::MakeScaleMatrix(scale, scale, scale, scaleMatrix);
		holo::utils::MakeScaleMatrix(1 / scale, 1 / scale, 1 / scale, scaleNegMatrix);

		holo::utils::MultMatrix(modelMatrix_sphere, translateMatrix, rotateMatrix);
		holo::utils::MultMatrix(invModelMatrix_sphere, rotateTransposeMatrix, translateNegMatrix);

		holo::utils::MultMatrix(modelMatrix_sphere, modelMatrix_sphere, scaleMatrix);
		holo::utils::MultMatrix(invModelMatrix_sphere, scaleNegMatrix, invModelMatrix_sphere);

		/* Transform world-space eye and light positions to sphere's object-space. */
		holo::utils::Transform(objSpaceLightPosition_sphere, invModelMatrix_sphere,	lightPosition);
		holo::utils::Transform(objSpaceEyePosition_sphere, invModelMatrix_sphere, eyePosition);

		// glEnable(GL_CULL_FACE);
		i = 0;
		j = 0;

		xpos = 0;
		h = 0;
		v = 0;

		//glClearColor(0.5,0.5,0.5,0.5);//JB Hack: clear view buffer to gray for debugging

		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

#if 1
		std::unique_lock<std::mutex> cloudLock(cloudMutex_);

		for (i = 0; i < numViews_; i++)
		{
			if ((viewEnableBitmask_ & (1 << i)) == 0)
			{
				continue;
			}

			glClear(GL_DEPTH_BUFFER_BIT);
			rgba = ((i / 4.0f) - int(i / 4.0f)) * 4.0f;
			h = int(i / (tileY_ * 4)) * numX_;
			v = (int(i / 4.0f) / ((float)tileY_) - int(int(i / 4.0f)
				/ ((float)tileY_))) * numY_ * tileY_;

			double q = tan((i - numViews_ / 2.0f) / numViews_ * fieldOfView_ / mag_ * M_PI/180.0f); //angle +/-fov/2

			//hologram is 150 mm wide, 75 mm high
			holo::utils::BuildShearOrthographicMatrix2(
				-hologramPlaneWidthMM_ / 2.0,
				hologramPlaneWidthMM_ / 2.0,
				-hologramPlaneHeightMM_ / 2.0,
				hologramPlaneHeightMM_ / 2.0,
				nearPlane_ * mag_,
				farPlane_* mag_,
				q,
				projectionMatrix1_);

			glViewport(h, v, numX_, numY_);
			enableDrawDepth_ = 0;

			drawScene(eyePosition, modelMatrix_sphere, invModelMatrix_sphere,
				objSpaceLightPosition_sphere, modelMatrix_cone,
				invModelMatrix_cone, objSpaceLightPosition_cone, h, v,
				enableDrawDepth_, 0, i);

			//glViewport(h,v+120*tiley+16,160,120); //1024-6*160=64, (512-120*2*2)/2=16
			glViewport(h, v + numY_ * tileY_, numX_, numY_); //setup viewport for depthbuffer render
			enableDrawDepth_ = 1;

			drawScene(eyePosition, modelMatrix_sphere, invModelMatrix_sphere,
				objSpaceLightPosition_sphere, modelMatrix_cone,
				invModelMatrix_cone, objSpaceLightPosition_cone, h, v,
				enableDrawDepth_, 0, i);



			//auto localFrameImg = cv::Mat(numY_, numX_, CV_8UC1);

			//glReadPixels(0, 0, numX_, numY_, GL_RED, GL_UNSIGNED_BYTE, localFrameImg.data);

			//cv::imshow("TextureView", localFrameImg);

			//std::stringstream ss;

			//ss << "c:\\temp\\img" << i << ".png";

			//cv::imwrite(ss.str(), localFrameImg);
		}

		haveNewCloud_.store(false);

		cloudLock.unlock();

#endif


		cgGLDisableProfile(normalMapLightingCgVertexProfile_);
		//checkForCgError("disabling vertex profile");
		cgGLDisableProfile(normalMapLightingCgFragmentProfile_);
		//checkForCgError("disabling fragment profile");

		//glDisable(GL_TEXTURE_2D);

		glPopAttrib();

		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, textureID_);
		//glFlush();
		//   glCopyTexSubImage			2D(GL_TEXTURE_2D, 0,0,0,0,0,imwidth,imheight);
		glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, viewTexWidth_, viewTexHeight_);
		//	printf("I'm here\n");
		//checkErrors();
		glBindTexture(GL_TEXTURE_2D, 0);
		glDisable(GL_TEXTURE_2D);

		//glEnable(GL_TEXTURE_2D);
		//glBindTexture(GL_TEXTURE_2D, textureID_[0]);

		//glFlush();
		//   glCopyTexSubImage			2D(GL_TEXTURE_2D, 0,0,0,0,0,imwidth,imheight);
		//glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, 0, 0, viewTexWidth_, viewTexHeight_);
		//	printf("I'm here\n");
		//glCheckErrors();
		//glBindTexture(GL_TEXTURE_2D, 0);
		//glDisable(GL_TEXTURE_2D);
#else //end of disable view render

	std::unique_lock<std::mutex> cloudLock(cloudMutex_);

	this->drawPointCloud();

	haveNewCloud_.store(false);
	cloudLock.unlock();

#endif

//Fringe computation
#if 0
		float quadRadius = 0.5;

		// glViewport(0,0,imwidth,512);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//glClear (GL_DEPTH_BUFFER_BIT);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(-quadRadius, quadRadius, -quadRadius, quadRadius, 0, 0.125);
		// glOrtho(-512,512-1,-256,256,1,125);
		// gluLookAt(0,0,0,0,0,-100,0,1,0);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		// glTranslatef(0.0,0.0,-100);
		//glViewport(0,0,1280,880);

		glViewport(0, 0, displayModeWidth_, displayModeHeight_);

		//glTranslatef(0.0, -0.25, 0.0); // JB: what does this do?
		glEnable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, textureID_);

		glDisable(GL_LIGHTING);

		cgGLBindProgram(cgVertexProgram_);
		//checkForCgError2("binding vertex program -fringes");
		cgGLEnableProfile(cgVertexProfile_);
		//checkForCgError2("enabling vertex profile -fringes");

		cgGLBindProgram(cgFragmentProgram_);
		//checkForCgError("binding fragment program");
		cgGLEnableProfile(cgFragmentProfile_);
		//checkForCgError("enabling fragment profile");

		cgGLEnableTextureParameter(cgFragmentParamDecal1_);
		//  checkForCgError2("enable decal texture0");
		//  cgGLEnableTextureParameter(myCgFragmentParam_decal1);
		//  checkForCgError2("enable decal texture1");

		//cgUpdateProgramParameters(myCgFragmentProgram2);

		glColor3f(1.0f, 1.0f, 1.0f);
		//   glutSolidTeapot(75);
		//	glTranslatef(0.0,0.0,-100.);
		glBegin(GL_QUADS);
		glNormal3f(0.0f, 0.0f, 1.0f);

		glTexCoord4f(0.0f, 1.0f, 0.0f, 1.0f);
		glVertex3f(-0.5f, 0.5f, 0.0f);

		glTexCoord4f(1.0f, 1.0f, 0.0f, 1.0f);
		glVertex3f(0.5f, 0.5f, 0.0f);

		glTexCoord4f(1.0f, 0.0f, 0.0f, 1);
		glVertex3f(0.5f, -0.5f, 0.0f);

		glTexCoord4f(0.0f, 0.0f, 0.0f, 1.0f);
		glVertex3f(-0.5f, -0.5f, 0.0f);

		glEnd();

		glDisable(GL_TEXTURE_2D);
		glBindTexture(GL_TEXTURE_2D, 0);

		cgGLDisableProfile(normalMapLightingCgVertexProfile_);
		checkForCgError("disabling vertex profile");
		cgGLDisableProfile(normalMapLightingCgFragmentProfile_);
		checkForCgError("disabling fragment profile");

		//glutPostRedisplay();

#endif
		glutSwapBuffers();
}

void HoloRenderDSCP2::cleanup()
{

}

void HoloRenderDSCP2::glutDisplay(void)
{
	gCurrentInstance->display();
}

void HoloRenderDSCP2::glutIdle(void)
{
	gCurrentInstance->idle();
}

void HoloRenderDSCP2::glutKeyboard(unsigned char c, int x, int y)
{
	gCurrentInstance->keyboard(c, x, y);
}

void HoloRenderDSCP2::glutCleanup(void)
{
	gCurrentInstance->cleanup();
}

void HoloRenderDSCP2::drawScene(float *eyePosition, float *modelMatrix_sphere,
	float *invModelMatrix_sphere, float *objSpaceLightPosition_sphere,
	float *modelMatrix_cone, float *invModelMatrix_cone,
	float *objSpaceLightPosition_cone, float h, float v, int drawdepthOn,
	float myobject, int viewnumber)
{
	// const float lightPosition[4] = { 10*mag+lx,20*mag+ly,-605*mag+lz, 1 };
	const float lightPosition[4] =	{ 1.00, 1.00, 6.05, 1 };
	//const float eyePosition[4] = { 0,0,0, 1 };

	float translateMatrix[16], rotateMatrix[16], viewMatrix[16],
		modelViewMatrix[16], modelViewProjMatrix[16];
	float objSpaceEyePosition[4], objSpaceLightPosition[4];

	holo::utils::BuildLookAtMatrix(eyePosition[0], eyePosition[1], eyePosition[2], 0, 0,-0.425f, 0, 1, 0, viewMatrix);

	/*** Render brass solid sphere ***/

//#ifndef VIEWS_FROM_CLOUD

	//setRedPlasticMaterial();
	//cgSetBrassMaterial();
	//setEmissiveLightColorOnly();

//#endif

	cgSetParameter1f(cgFragmentParamDrawdepth_, drawdepthOn);
	/* Transform world-space eye and light positions to sphere's object-space. */
	//transform(objSpaceEyePosition, invModelMatrix_sphere, eyePosition);

	cgSetParameter3fv(cgFragmentParamEyePosition_, objSpaceEyePosition);

	//transform(objSpaceLightPosition, invModelMatrix_sphere, lightPosition);
	cgSetParameter3fv(cgFragmentParamLightPosition_, objSpaceLightPosition_sphere);

	holo::utils::MultMatrix(modelViewProjMatrix, projectionMatrix1_, modelMatrix_sphere);

	//holo::utils::MultMatrix(modelViewMatrix, viewMatrix, modelMatrix_sphere);
	//holo::utils::MultMatrix(modelViewProjMatrix, projectionMatrix1_, modelViewMatrix);

	// glEnable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	//glBindFramebufferEXT(GL_FRAMEBUFFER_EXT, fbo); // bind the frame buffer object
	//glViewport(h,v,64,440);

	/* Set matrix parameter with row-major matrix. */
	cgSetMatrixParameterfr(cgVertexParamModelViewProj_, modelViewProjMatrix);
	cgUpdateProgramParameters(normalMapLightingCgVertexProgram_);

	this->drawPointCloud();

	//glDisable(GL_TEXTURE_2D);

	//this->drawPointCloud();

}

void HoloRenderDSCP2::drawPointCloud()
{
	//if (haveNewCloud_.load())
	//{
		//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		//std::lock_guard<std::mutex> lg(cloudMutex_);
		const int ystride = 1; //only render every ystride lines
		const float gain = 1 / 256.0; // converting from char units to float

		//glEnable(GL_TEXTURE_2D);

		glEnable(GL_POINT_SMOOTH);
		glPointSize(1.0f);
		//float attenparams[3] = {0,0,0}; //a b c	//size × 1 a + b × d + c × d 2
		//glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION,attenparams);
		glBegin(GL_POINTS);

		HoloPoint3D *pointIdx = cloud_->points.data();
		float luma = 0.0f;
		for (int i = 0; i < cloud_->size(); i+=ystride)
		{
			if (pointIdx->z == HOLO_CLOUD_BAD_POINT)
				continue;

			luma = (pointIdx->r + pointIdx->g + pointIdx->b)/3 * gain;
			glVertex4f(pointIdx->x, pointIdx->y, pointIdx->z, 1.0f);
			glColor3f(luma, luma, luma);
			pointIdx+=ystride;
		}

		glEnd();

		glDisable(GL_POINT_SMOOTH);
		//glDisable(GL_TEXTURE_2D);
		
		//haveNewCloud_.store(false);

		//glutPostRedisplay();
	//}
}

void HoloRenderDSCP2::drawString(float posX, float posY, std::string theString)
{
	glRasterPos2f(posX, posY);

	for (int i = 0; i < theString.size(); i++)
	{
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, theString[i]);
	}
}

bool HoloRenderDSCP2::checkForCgErrorLine(const char *situation, int line)
{
	CGerror error;
	const char *string = cgGetLastErrorString(&error);

	if (error != CG_NO_ERROR)
	{
		LOG4CXX_ERROR(logger_, situation << ": " << string);
		if (error == CG_COMPILER_ERROR)
		{
			LOG4CXX_ERROR(logger_, "Cg compiler error: " << cgGetLastListing(normalMapLightingCgContext_));
			exit(-1);
		}
		
		return true;
	}

	return false;
}

bool HoloRenderDSCP2::checkForCgError(const char *situation)
{
	return checkForCgErrorLine(situation, __LINE__);
}

bool HoloRenderDSCP2::checkForCgError2(const char *situation)
{
	CGerror error;
	const char *string = cgGetLastErrorString(&error);

	if (error != CG_NO_ERROR)
	{
		LOG4CXX_ERROR(logger_, situation << ": " << string);
		if (error == CG_COMPILER_ERROR)
		{
			LOG4CXX_ERROR(logger_, "Cg compiler error: " << cgGetLastListing(normalMapLightingCgContext_));
		}

		return true;
	}

	return false;
}

void HoloRenderDSCP2::cgSetBrassMaterial(){
	const float brassEmissive[3] =
	{ 0.0, 0.0, 0.0 }, brassAmbient[3] =
	{ 0.33 * 2, 0.33 * 2, 0.33 * 2 }, brassDiffuse[3] =
	{ 0.78, 0.78, 0.78 }, brassSpecular[3] =
	{ 0.99, 0.99, 0.99 }, brassShininess = 27.8;

	cgSetParameter3fv(cgFragmentParamKe_, brassEmissive);
	cgSetParameter3fv(cgFragmentParamKa_, brassAmbient);
	cgSetParameter3fv(cgFragmentParamKd_, brassDiffuse);
	cgSetParameter3fv(cgFragmentParamKs_, brassSpecular);
	cgSetParameter1f(cgFragmentParamShininess_, brassShininess);
}

void HoloRenderDSCP2::cgSetRedPlasticMaterial()
{
	const float redPlasticEmissive[3] =
	{ 0.0, 0.0, 0.0 }, redPlasticAmbient[3] =
	{ 0.2, 0.2, 0.2 }, redPlasticDiffuse[3] =
	{ 0.5, 0.5, 0.5 }, redPlasticSpecular[3] =
	{ 0.6, 0.6, 0.6 }, redPlasticShininess = 32.0;

	cgSetParameter3fv(cgFragmentParamKe_, redPlasticEmissive);
	checkForCgError("setting Ke parameter");
	cgSetParameter3fv(cgFragmentParamKa_, redPlasticAmbient);
	checkForCgError("setting Ka parameter");
	cgSetParameter3fv(cgFragmentParamKd_, redPlasticDiffuse);
	checkForCgError("setting Kd parameter");
	cgSetParameter3fv(cgFragmentParamKs_, redPlasticSpecular);
	checkForCgError("setting Ks parameter");
	cgSetParameter1f(cgFragmentParamShininess_, redPlasticShininess);
	checkForCgError("setting shininess parameter");
}

void HoloRenderDSCP2::cgSetEmissiveLightColorOnly()
{
	const float zero[3] = { 0.0, 0.0, 0.0 };

	cgSetParameter3fv(cgFragmentParamKe_, lightColor_);
	checkForCgError("setting Ke parameter");
	cgSetParameter3fv(cgFragmentParamKa_, zero);
	checkForCgError("setting Ka parameter");
	cgSetParameter3fv(cgFragmentParamKd_, zero);
	checkForCgError("setting Kd parameter");
	cgSetParameter3fv(cgFragmentParamKs_, zero);
	checkForCgError("setting Ks parameter");
	cgSetParameter1f(cgFragmentParamShininess_, 0);
	checkForCgError("setting shininess parameter");
}

void HoloRenderDSCP2::deinit()
{
	
}

void HoloRenderDSCP2::updateFromPointCloud(HoloCloudPtr && pointCloud)
{
	std::lock_guard<std::mutex> lg(cloudMutex_);
	cloud_ = std::move(pointCloud);
	haveNewCloud_.store(true);
}

#endif