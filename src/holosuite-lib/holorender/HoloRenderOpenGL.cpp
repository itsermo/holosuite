#include "HoloRenderOpenGL.hpp"

using namespace holo;
using namespace holo::render;

HoloRenderOpenGL::HoloRenderOpenGL() : HoloRenderOpenGL(HOLO_RENDER_OPENGL_ENABLE_ZSPACE_RENDERING, HOLO_RENDER_OPENGL_DEFAULT_VOXEL_SIZE)
{

}

HoloRenderOpenGL::HoloRenderOpenGL(int voxelSize, bool enableZSpaceRendering) :
	IHoloRender(),
	enableZSpaceRendering_(enableZSpaceRendering),
	voxelSize_(voxelSize),
	localCloud_(nullptr),
	remoteCloud_(nullptr),
	isInit_(false),
	firstInit_(true),
	haveNewRemoteCloud_(false),
	haveNewLocalCloud_(false),
	mouseLeftButton_(0),
	mouseMiddleButton_(0),
	mouseRightButton_(0),
	mouseDownX_(0),
	mouseDownY_(0),
	windowWidth_(640),
	windowHeight_(480),
	viewPhi_(0.0f),
	viewTheta_(0.0f),
	viewDepth_(0.0f)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.opengl");
	LOG4CXX_DEBUG(logger_, "Instantiating HoloRenderOpenGL object...");
	gCurrentOpenGLInstance = this;
	LOG4CXX_DEBUG(logger_, "Done instantiating HoloRenderOpenGL object");
}

HoloRenderOpenGL::~HoloRenderOpenGL()
{
	LOG4CXX_DEBUG(logger_, "Destroying HoloRenderOpenGL object...");
	LOG4CXX_DEBUG(logger_, "Done destroying HoloRenderOpenGL object");
}

bool HoloRenderOpenGL::init()
{
	LOG4CXX_INFO(logger_, "Initializing OpenGL render algorithm...");

	localCloud_ = HoloCloudPtr(new HoloCloud);
	remoteCloud_ = HoloCloudPtr(new HoloCloud);

	glutInitThread_ = std::thread(&HoloRenderOpenGL::glutInitLoop, this);

	std::unique_lock<std::mutex> lg(hasInitMutex_);
	hasInitCV_.wait(lg);

	return isInit_;
}

void HoloRenderOpenGL::deinit()
{

}


void HoloRenderOpenGL::glutInitLoop()
{
	GLenum glError = GL_NO_ERROR;

#ifdef WIN32
	//char fakeParam[] = "dscp2";
	const char *fakeargv[] = { "holosuite", NULL };
	int fakeargc = 1;
#elif defined(__linux) || defined(__unix) || defined(__posix)
	char displayEnvArg[1024];
	strncpy(displayEnvArg, displayEnv_.c_str(), displayEnv_.length());

	const char *fakeargv[] = { "holosuite", "-display", displayEnvArg, NULL };
	int fakeargc = 3;
#endif

	//glCheckErrors();

	// Initialize GLUT window and callbacks
	glutInit(&fakeargc, const_cast<char**>(fakeargv));
	glutInitWindowSize(windowWidth_, windowHeight_);
	glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH);

	std::stringstream ss;
	ss << "holosuite";
	glutCreateWindow(ss.str().c_str());

	glCheckErrors();

	glutDisplayFunc(this->glutDisplay);
	glutKeyboardFunc(this->glutKeyboard);
	glutIdleFunc(this->glutIdle);
	glutMouseFunc(this->glutMouse);
	glutMotionFunc(this->glutMouseMotion);
	glutReshapeFunc(this->glutReshape);
	atexit(this->glutCleanup);

	// GLUT settings
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Black Background
	glShadeModel(GL_SMOOTH); // Enable Smooth Shading

	//glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient_); // Setup The Ambient Light
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse_); // Setup The Diffuse Light
	//glLightfv(GL_LIGHT0, GL_POSITION, lightPosition_); // Position The Light
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST); // Enables Depth Testing
	glEnable(GL_COLOR_MATERIAL);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

	//glClearColor(0, 0, 0, 0);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, static_cast<float>(windowWidth_) / static_cast<float>(windowHeight_), 0.01f, 10.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glDisable(GL_LIGHTING);

	glCheckErrors();

	glutMainLoop();
}

void HoloRenderOpenGL::glCheckErrors()
{
	GLenum error;

	while ((error = glGetError()) != GL_NO_ERROR)
	{
		LOG4CXX_WARN(logger_, "OpenGL error: " << gluErrorString(error));
	}
}

void HoloRenderOpenGL::reshape(int width, int height)
{
	windowWidth_ = width;
	windowHeight_ = height;
	glutPostRedisplay();
}

void HoloRenderOpenGL::idle()
{
	// refresh point cloud data
	if (haveNewRemoteCloud_.load())
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

		LOG4CXX_TRACE(logger_, "DSCP2 rendering on head " << headNum_ << " @ " << fps << " fps");
		//		fflush(stdout);
	}

#endif
}

void HoloRenderOpenGL::keyboard(unsigned char c, int x, int y)
{
	//printf("keyboard \n");
	switch (c)
	{

	//case 'z': //invert set of disabled views
	//	viewEnableBitmask_ = ~viewEnableBitmask_;
	//	break;
	//case 'Z': //enable all views
	//	viewEnableBitmask_ = -1;
	//	break;
	//case 'x': //cycle through debug modes in shader (output intermediate variables)
	//	hologramOutputDebugSwitch_++;
	//	hologramOutputDebugSwitch_ = hologramOutputDebugSwitch_ % HOLO_RENDER_DSCP2_NUM_DEBUG_SWITCHES;
	//	cgSetParameter1f(fringeFragmentArgs_.debugSwitch, hologramOutputDebugSwitch_);
	//	break;
	//case 'X':
	//	hologramOutputDebugSwitch_--;
	//	hologramOutputDebugSwitch_ = hologramOutputDebugSwitch_ % HOLO_RENDER_DSCP2_NUM_DEBUG_SWITCHES;
	//	cgSetParameter1f(fringeFragmentArgs_.debugSwitch, hologramOutputDebugSwitch_);
	//	break;
	//case 'j':
	//	translateX_ = translateX_ + 0.01;
	//	printf("tx %f \n", translateX_);
	//	break;
	//case 'l':
	//	translateX_ = translateX_ - 0.01;
	//	printf("tx %f \n", translateX_);
	//	break;
	//case 'i':
	//	translateY_ = translateY_ - 0.01;
	//	printf("ty %f \n", translateY_);
	//	break;
	//case 'k':
	//	translateY_ = translateY_ + 0.01;
	//	printf("ty %f \n", translateY_);
	//	break;

	//case 'w':
	//	translateZ_ = translateZ_ - 0.01;
	//	printf("%f \n", translateZ_ - 0.675);
	//	break;
	//case 's':
	//	translateZ_ = translateZ_ + 0.01;
	//	printf("%f \n", translateZ_ - 0.675);
	//	break;

	//case 'f':
	//	//writeToFile2();
	//	break;
	//case 'F':
	//	//printf("writing view texture\n");
	//	//writeViewsToFile();
	//	break;
	//case 'e':
	//	translateZ_ = translateZ_ - 0.01;
	//	printf("%f \n", translateZ_ - 675);
	//	break;
	//case 'd':
	//	translateZ_ = translateZ_ + 0.01;
	//	printf("%f \n", translateZ_ - 675);
	//	break;
	//case 'c':
	//	translateZ_ = 0;
	//	printf("%f \n", translateZ_ - 675);
	//	break;
	//case 'r':
	//	rot_ = rot_ - 5;
	//	rotateCounter_ = (rotateCounter_ * -1) + 1;

	//	printf("rotate %i \n", rotateCounter_);
	//	break;
	//case ' ':
	//	//makeViewtexFromFile();
	//	break;
	//case ']':
	//	lightLocationZ_ = lightLocationZ_ - 1.0f;
	//	printf("%f \n", lightLocationZ_);
	//	break;
	//case '[':
	//	lightLocationZ_ = lightLocationZ_ + 1.0f;
	//	printf("%f \n", lightLocationZ_);
	//	break;
	//case '=':
	//	lightLocationY_ = lightLocationY_ - 1.0f;
	//	printf("%f \n", lightLocationY_);
	//	break;
	//case '-':
	//	lightLocationY_ = lightLocationY_ + 1.0f;
	//	printf("%f \n", lightLocationY_);
	//	break;
	//case ';':
	//	lightLocationX_ = lightLocationX_ - 1.0f;
	//	printf("%f \n", lightLocationX_);
	//	break;
	//case '/':
	//	lightLocationX_ = lightLocationX_ + 1.0f;
	//	printf("%f \n", lightLocationX_);
	//	break;
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
		// ShutDown();
		exit(0);
		break;
	}

	//int mods = glutGetModifiers();
	//if (mods != 0)
	//{
	//	if (c >= '0' && c <= '9') viewEnableBitmask_ ^= 1 << (c - '0' + 10); //toggle view enable bit for numbered view 10-19 (only 16 views used)
	//}
	//else {
	//	if (c >= '0' && c <= '9') viewEnableBitmask_ ^= 1 << (c - '0'); //toggle view enable bit for numbered view 0-9

	//}

	glutPostRedisplay();
}

void HoloRenderOpenGL::display()
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

	std::unique_lock<std::mutex> cloudLock(remoteCloudMutex_);

	this->drawPointCloud();

	haveNewRemoteCloud_.store(false);
	cloudLock.unlock();

	glViewport(0, 0, (GLsizei)windowWidth_, (GLsizei)windowHeight_);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, static_cast<float>(windowWidth_) / static_cast<float>(windowHeight_), 0.01f, 10.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 1);
	glTranslatef(0.0, 0.0, viewDepth_);
	glRotatef(-viewTheta_, 1.0, 0.0, 0.0);
	glRotatef(-viewPhi_, 0.0, 1.0, 0.0);

	glutSwapBuffers();
}

void HoloRenderOpenGL::cleanup()
{

}

void HoloRenderOpenGL::mouse(int button, int state, int x, int y)
{
	mouseDownX_ = x; mouseDownY_ = y;
	mouseLeftButton_ = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN));
	mouseMiddleButton_ = ((button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN));
	glutPostRedisplay();
}

void HoloRenderOpenGL::mouseMotion(int x, int y)
{
	if (mouseLeftButton_){ viewPhi_ += (float)(x - mouseDownX_) / 4.0; viewTheta_ += (float)(mouseDownY_ - y) / 4.0; } // rotate
	if (mouseMiddleButton_){ viewDepth_ += (float)(mouseDownY_ - y) / 10.0; } // scale
	mouseDownX_ = x;   mouseDownY_ = y;
	glutPostRedisplay();
}

void HoloRenderOpenGL::glutReshape(int width, int height)
{
	gCurrentOpenGLInstance->reshape(width, height);
}

void HoloRenderOpenGL::glutDisplay(void)
{
	gCurrentOpenGLInstance->display();
}

void HoloRenderOpenGL::glutIdle(void)
{
	gCurrentOpenGLInstance->idle();
}

void HoloRenderOpenGL::glutKeyboard(unsigned char c, int x, int y)
{
	gCurrentOpenGLInstance->keyboard(c, x, y);
}

void HoloRenderOpenGL::glutMouse(int button, int state, int x, int y) 
{
	gCurrentOpenGLInstance->mouse(button, state, x, y);
}

void HoloRenderOpenGL::glutMouseMotion(int x, int y)
{
	gCurrentOpenGLInstance->mouseMotion(x, y);
}

void HoloRenderOpenGL::glutCleanup(void)
{
	gCurrentOpenGLInstance->cleanup();
}

void HoloRenderOpenGL::updateLocalPointCloud(HoloCloudPtr && pointCloud)
{

}

void HoloRenderOpenGL::updateRemotePointCloud(HoloCloudPtr && pointCloud)
{
	std::lock_guard<std::mutex> lg(remoteCloudMutex_);
	remoteCloud_ = std::move(pointCloud);
	haveNewRemoteCloud_.store(true);
}

void HoloRenderOpenGL::drawPointCloud()
{
	//if (haveNewCloud_.load())
	//{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	//std::lock_guard<std::mutex> lg(cloudMutex_);
	const int ystride = 1; //only render every ystride lines
	const float gain = 1 / 255.0; // converting from char units to float

	//glEnable(GL_TEXTURE_2D);

	glEnable(GL_POINT_SMOOTH);
	glPointSize(1.0f);
	//float attenparams[3] = {0,0,0}; //a b c	//size ? 1 a + b ? d + c ? d 2
	//glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION,attenparams);
	glBegin(GL_POINTS);

	HoloPoint3D *pointIdx = remoteCloud_->points.data();
	float luma = 0.0f;
	for (int i = 0; i < remoteCloud_->size(); i += ystride)
	{
		if (pointIdx->z == HOLO_CLOUD_BAD_POINT)
			continue;

		//luma = (pointIdx->r + pointIdx->g + pointIdx->b) / 3 * gain;
		glVertex4f(pointIdx->x, pointIdx->y, pointIdx->z, 1.0f);
		glColor3f(pointIdx->r * gain, pointIdx->g * gain, pointIdx->b * gain);
		pointIdx += ystride;
	}

	glEnd();

	glDisable(GL_POINT_SMOOTH);
	//glDisable(GL_TEXTURE_2D);

	//haveNewCloud_.store(false);

	//glutPostRedisplay();
	//}
}


void* HoloRenderOpenGL::getContext()
{
	return nullptr;
}