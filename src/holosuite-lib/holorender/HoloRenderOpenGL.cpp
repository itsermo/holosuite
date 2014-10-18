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

#ifdef ENABLE_HOLO_ZSPACE

	ZSError error;

	// Initialize the zSpace SDK. This MUST be called before
	// calling any other zSpace API.
	error = zsInitialize(&zSpaceContext_);
	CHECK_ERROR(error);

	// Create a stereo buffer to handle L/R detection.
	error = zsCreateStereoBuffer(zSpaceContext_, ZS_RENDERER_QUAD_BUFFER_GL, 0, &bufferHandle_);
	CHECK_ERROR(error);

	// Create a zSpace viewport object and grab its associated frustum. 
	// Note: The zSpace viewport is abstract and not an actual window/viewport
	// that is created and registered through the Windows OS. It manages
	// a zSpace stereo frustum, which is responsible for various stereoscopic 
	// 3D calculations such as calculating the view and projection matrices for 
	// each eye.
	error = zsCreateViewport(zSpaceContext_, &viewportHandle_);
	CHECK_ERROR(error);

	error = zsFindFrustum(viewportHandle_, &frustumHandle_);
	CHECK_ERROR(error);

	// Grab a handle to the stylus target.
	error = zsFindTargetByType(zSpaceContext_, ZS_TARGET_TYPE_PRIMARY, 0, &stylusHandle_);
	CHECK_ERROR(error);

	// Find the zSpace display and set the window's position
	// to be the top left corner of the zSpace display.
	error = zsFindDisplayByType(zSpaceContext_, ZS_DISPLAY_TYPE_ZSPACE, 0, &displayHandle_);
	CHECK_ERROR(error);

	error = zsGetDisplayPosition(displayHandle_, &windowX_, &windowY_);
	CHECK_ERROR(error);
#endif

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

	if (enableZSpaceRendering_)
		glutInitDisplayMode(GLUT_RGBA | GLUT_ALPHA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STEREO);
	else
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
	//glMatrixMode(GL_PROJECTION);
	//glLoadIdentity();
	//gluPerspective(45.0f, static_cast<float>(windowWidth_) / static_cast<float>(windowHeight_), 0.01f, 10.0f);
	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();
	//gluLookAt(0, 0, 0, 0, 0, 1, 0, 1, 1);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0.0f, 0.345f, 0.222f,   // Eye
		0.0f, 0.0f, 0.0f,     // Center
		0.0f, 1.0f, 0.0f);    // Up 

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
	//if (haveNewRemoteCloud_.load())
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

	case 'f':
		glutFullScreen();
		//writeToFile2();
		break;
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

#ifdef ENABLE_HOLO_ZSPACE
	if (enableZSpaceRendering_)
	{
		update();
		draw();
	}
	else
	{
#endif
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

		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		std::unique_lock<std::mutex> cloudLock(remoteCloudMutex_);

		this->drawPointCloud();

		haveNewRemoteCloud_.store(false);
		cloudLock.unlock();

		glutSwapBuffers();

#ifdef ENABLE_HOLO_ZSPACE
	}
#endif

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
	const float gain = 1 / 255.0; // converting from char units to float

	//glEnable(GL_TEXTURE_2D);

	glEnable(GL_POINT_SMOOTH);
	glPointSize(voxelSize_*4);
	//float attenparams[3] = {0,0,0}; //a b c	//size ? 1 a + b ? d + c ? d 2
	//glPointParameterfv(GL_POINT_DISTANCE_ATTENUATION,attenparams);
	glBegin(GL_POINTS);

	HoloPoint3D *pointIdx = remoteCloud_->points.data();
	float luma = 0.0f;
	for (int i = 0; i < remoteCloud_->size(); i++)
	{
		if (pointIdx->z == HOLO_CLOUD_BAD_POINT)
			continue;

		//luma = (pointIdx->r + pointIdx->g + pointIdx->b) / 3 * gain;
		glVertex4f(pointIdx->x+0.1, pointIdx->y + 0.12, pointIdx->z - 0.05, 1.0f);
		glColor3f(pointIdx->r * gain, pointIdx->g * gain, pointIdx->b * gain);
		pointIdx++;
	}

	glEnd();

	glDisable(GL_POINT_SMOOTH);

	glColor3f(.3, .3, .3);
	glBegin(GL_QUADS);
	glVertex3f(-5, -1.01, 0);
	glVertex3f(-5, -1.01, 5);
	glVertex3f(5, -1.01, 5);
	glVertex3f(5, -1.01, 0);
	glEnd();

	glColor3f(.3, .3, .3);
	glBegin(GL_QUADS);
	glVertex3f(-5, -1.01, 5);
	glVertex3f(-5, 1.01, 5);
	glVertex3f(5, 1.01, 5);
	glVertex3f(5, -1.01, 5);
	glEnd();

	glColor3f(.3, .3, .3);
	glBegin(GL_QUADS);
	glVertex3f(-5, 1.01, 5);
	glVertex3f(-5, 1.01, 0);
	glVertex3f(5, 1.01, 0);
	glVertex3f(5, 1.01, 5);
	glEnd();

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(6);

	glBegin(GL_LINES);

	for (int i = -5; i <= 5; i++) {
		if (i == 0) { glColor3f(.6, .3, .3); }
		else { glColor3f(.6, .6, .25); };
		glVertex3f(i, -1, -5);
		glVertex3f(i, -1, 5);
		if (i == 0) { glColor3f(.3, .3, .6); }
		else { glColor3f(.6, .6, .25); };
		glVertex3f(-5, -1, i);
		glVertex3f(5, -1, i);
		//glVertex3f(-10, -1, i);
	};

	for (int i = -5; i <= 5; i++) {
		glColor3f(.6, .6, .25);
		glVertex3f(-5, 1, i);
		glVertex3f(5, 1, i);
		glColor3f(.6, .6, .25);
		glVertex3f(i, 1, -5);
		glVertex3f(i, 1, 5);
		//glVertex3f(-10, -1, i);
	};

	for (int i = -5; i <= 5; i++) {
		glColor3f(.6, .6, .25);
		glVertex3f(-5, i, 5);
		glVertex3f(5, i, 5);
		glColor3f(.6, .6, .25);
		glVertex3f(i, -1, 5);
		glVertex3f(i, 1, 5);
		//glVertex3f(-10, -1, i);
	};

	glDisable(GL_LINE_SMOOTH);

	glEnd();

	//glDisable(GL_TEXTURE_2D);

	//haveNewCloud_.store(false);

	//glutPostRedisplay();
	//}
}


void* HoloRenderOpenGL::getContext()
{
	return nullptr;
}

#ifdef ENABLE_HOLO_ZSPACE

bool HoloRenderOpenGL::update()
{
	ZSError error;

	// Update the camera.
	updateCamera();

	// Update the zSpace viewport position and size based
	// on the position and size of the application window.
	error = zsSetViewportPosition(viewportHandle_, windowX_, windowY_);
	CHECK_ERROR(error);

	error = zsSetViewportSize(viewportHandle_, windowWidth_, windowHeight_);
	CHECK_ERROR(error);

	// Update the OpenGL viewport size;
	glViewport(0, 0, windowWidth_, windowHeight_);

	// Update the zSpace SDK. This updates both tracking information
	// as well as the head poses for any frustums that have been created.
	error = zsUpdate(zSpaceContext_);
	CHECK_ERROR(error);

	//// Grab the stylus pose (position and orientation) in tracker space.
	//ZSTrackerPose stylusPose;
	//error = zsGetTargetPose(stylusHandle, &stylusPose);
	//CHECK_ERROR(error);

	//// Transform the stylus pose from tracker to camera space.
	//error = zsTransformMatrix(viewportHandle_, ZS_COORDINATE_SPACE_TRACKER, ZS_COORDINATE_SPACE_CAMERA, &stylusPose.matrix);
	//CHECK_ERROR(error);

	// Transform the stylus pose from camera space to world space.
	// This is done by multiplying the pose (camera space) by the 
	// application's camera matrix.
	//matrixMultiply(cameraMatrix_.f, stylusPose.matrix.f, g_stylusWorldMatrix.f);

	return true;
}

void HoloRenderOpenGL::updateCamera()
{
	// Calculate the camera's new position such that it orbits
	// the world's origin.
	GLfloat eyeX = 0.222f * sin(cameraAngle_ * PI / 180.0f);
	GLfloat eyeY = 0.345f;
	GLfloat eyeZ = 0.222f * cos(cameraAngle_ * PI / 180.0f);

	// Use gluLookAt to calculate the new model-view matrix.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eyeX, eyeY, eyeZ,   // Eye
		0.0f, 0.0f, 1.0f,   // Center  
		0.0f, 1.0f, 1.0f);  // Up

	// Grab the model-view matrix so that we can reference it later.
	glGetFloatv(GL_MODELVIEW_MATRIX, modelViewMatrix_.f);

	// Invert the model-view matrix to obtain the camera's transformation matrix.
	matrixInverse(modelViewMatrix_.f, cameraMatrix_.f);

	// Update the camera angle if camera orbit is enabled.
	clock_t currentTime = clock();

	if (isCameraOrbitEnabled_)
	{
		float deltaTime = static_cast<float>(currentTime - previousTime_) / CLOCKS_PER_SEC;
		cameraAngle_ += ROTATION_PER_SECOND * deltaTime;
		cameraAngle_ = fmodf(cameraAngle_, 360.0f);
	}

	previousTime_ = currentTime;
}

void HoloRenderOpenGL::drawSceneForEye(ZSEye eye)
{
	// Push the stereo view and projection matrices onto the OpenGL matrix 
	// stack so that we can pop them off after we're done rendering the 
	// scene for a specified eye.  This will allow us to restore the mono 
	// (non-stereoscopic) model-view and projection matrices.
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();

	// Set the view and projection matrices for the specified eye.
	setViewMatrix(eye);
	setProjectionMatrix(eye);

	// Set the render target based for the specified eye.
	setRenderTarget(eye);

	// Clear the scene - color and depth buffers.
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	this->drawPointCloud();
	////// Draw the cube.
	////drawCube();

	////// Draw the stylus.
	////drawStylus();

	// Restore the mono (non-stereoscopic) model-view and projection matrices.
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void HoloRenderOpenGL::setRenderTarget(ZSEye eye)
{
	// Select appropriate back buffer to render to based on the specified eye.
	switch (eye)
	{
	case ZS_EYE_LEFT:
		glDrawBuffer(GL_BACK_LEFT);
		break;
	case ZS_EYE_RIGHT:
		glDrawBuffer(GL_BACK_RIGHT);
		break;
	default:
		break;
	}
}


bool HoloRenderOpenGL::setViewMatrix(ZSEye eye)
{
	// Get the view matrix from the zSpace StereoFrustum for the specified eye.
	ZSMatrix4 viewMatrix;
	ZSError error = zsGetFrustumViewMatrix(frustumHandle_, eye, &viewMatrix);
	CHECK_ERROR(error);

	// Set the model-view matrix for the specified eye and multiply it by the
	// mono (non-stereoscopic) model-view matrix.  This must be done because the
	// eye's view matrix only contains the eye offset (relative to the center of
	// the viewer's head) plus any rotation required for off-axis projection.
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(viewMatrix.f);
	glMultMatrixf(modelViewMatrix_.f);

	return true;
}


bool HoloRenderOpenGL::setProjectionMatrix(ZSEye eye)
{
	// Get the projection matrix from the zSpace StereoFrustum for a specified eye.
	ZSMatrix4 projectionMatrix;
	ZSError error = zsGetFrustumProjectionMatrix(frustumHandle_, eye, &projectionMatrix);
	CHECK_ERROR(error);

	// Set OpenGL MatrixMode to GL_PROJECTION and set the projection matrix.
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(projectionMatrix.f);

	return true;
}

void HoloRenderOpenGL::matrixMultiply(const float a[16], const float b[16], float r[16])
{
	r[0] = a[0] * b[0] + a[4] * b[1] + a[8] * b[2] + a[12] * b[3];
	r[4] = a[0] * b[4] + a[4] * b[5] + a[8] * b[6] + a[12] * b[7];
	r[8] = a[0] * b[8] + a[4] * b[9] + a[8] * b[10] + a[12] * b[11];
	r[12] = a[0] * b[12] + a[4] * b[13] + a[8] * b[14] + a[12] * b[15];

	r[1] = a[1] * b[0] + a[5] * b[1] + a[9] * b[2] + a[13] * b[3];
	r[5] = a[1] * b[4] + a[5] * b[5] + a[9] * b[6] + a[13] * b[7];
	r[9] = a[1] * b[8] + a[5] * b[9] + a[9] * b[10] + a[13] * b[11];
	r[13] = a[1] * b[12] + a[5] * b[13] + a[9] * b[14] + a[13] * b[15];

	r[2] = a[2] * b[0] + a[6] * b[1] + a[10] * b[2] + a[14] * b[3];
	r[6] = a[2] * b[4] + a[6] * b[5] + a[10] * b[6] + a[14] * b[7];
	r[10] = a[2] * b[8] + a[6] * b[9] + a[10] * b[10] + a[14] * b[11];
	r[14] = a[2] * b[12] + a[6] * b[13] + a[10] * b[14] + a[14] * b[15];

	r[3] = a[3] * b[0] + a[7] * b[1] + a[11] * b[2] + a[15] * b[3];
	r[7] = a[3] * b[4] + a[7] * b[5] + a[11] * b[6] + a[15] * b[7];
	r[11] = a[3] * b[8] + a[7] * b[9] + a[11] * b[10] + a[15] * b[11];
	r[15] = a[3] * b[12] + a[7] * b[13] + a[11] * b[14] + a[15] * b[15];
}


void HoloRenderOpenGL::matrixInverse(const float m[16], float i[16])
{
	float m00 = m[0], m01 = m[4], m02 = m[8], m03 = m[12];
	float m10 = m[1], m11 = m[5], m12 = m[9], m13 = m[13];
	float m20 = m[2], m21 = m[6], m22 = m[10], m23 = m[14];
	float m30 = m[3], m31 = m[7], m32 = m[11], m33 = m[15];

	float v0 = m20 * m31 - m21 * m30;
	float v1 = m20 * m32 - m22 * m30;
	float v2 = m20 * m33 - m23 * m30;
	float v3 = m21 * m32 - m22 * m31;
	float v4 = m21 * m33 - m23 * m31;
	float v5 = m22 * m33 - m23 * m32;

	float t00 = +(v5 * m11 - v4 * m12 + v3 * m13);
	float t10 = -(v5 * m10 - v2 * m12 + v1 * m13);
	float t20 = +(v4 * m10 - v2 * m11 + v0 * m13);
	float t30 = -(v3 * m10 - v1 * m11 + v0 * m12);

	float invDet = 1 / (t00 * m00 + t10 * m01 + t20 * m02 + t30 * m03);

	float d00 = t00 * invDet;
	float d10 = t10 * invDet;
	float d20 = t20 * invDet;
	float d30 = t30 * invDet;

	float d01 = -(v5 * m01 - v4 * m02 + v3 * m03) * invDet;
	float d11 = +(v5 * m00 - v2 * m02 + v1 * m03) * invDet;
	float d21 = -(v4 * m00 - v2 * m01 + v0 * m03) * invDet;
	float d31 = +(v3 * m00 - v1 * m01 + v0 * m02) * invDet;

	v0 = m10 * m31 - m11 * m30;
	v1 = m10 * m32 - m12 * m30;
	v2 = m10 * m33 - m13 * m30;
	v3 = m11 * m32 - m12 * m31;
	v4 = m11 * m33 - m13 * m31;
	v5 = m12 * m33 - m13 * m32;

	float d02 = +(v5 * m01 - v4 * m02 + v3 * m03) * invDet;
	float d12 = -(v5 * m00 - v2 * m02 + v1 * m03) * invDet;
	float d22 = +(v4 * m00 - v2 * m01 + v0 * m03) * invDet;
	float d32 = -(v3 * m00 - v1 * m01 + v0 * m02) * invDet;

	v0 = m21 * m10 - m20 * m11;
	v1 = m22 * m10 - m20 * m12;
	v2 = m23 * m10 - m20 * m13;
	v3 = m22 * m11 - m21 * m12;
	v4 = m23 * m11 - m21 * m13;
	v5 = m23 * m12 - m22 * m13;

	float d03 = -(v5 * m01 - v4 * m02 + v3 * m03) * invDet;
	float d13 = +(v5 * m00 - v2 * m02 + v1 * m03) * invDet;
	float d23 = -(v4 * m00 - v2 * m01 + v0 * m03) * invDet;
	float d33 = +(v3 * m00 - v1 * m01 + v0 * m02) * invDet;

	i[0] = d00; i[4] = d01; i[8] = d02; i[12] = d03;
	i[1] = d10; i[5] = d11; i[9] = d12; i[13] = d13;
	i[2] = d20; i[6] = d21; i[10] = d22; i[14] = d23;
	i[3] = d30; i[7] = d31; i[11] = d32; i[15] = d33;
}

void HoloRenderOpenGL::draw()
{
	// This must be called every frame on the rendering thread in order 
	// to handle the initial sync and any subsequent pending sync requests 
	// for left/right frame detection.
	ZSError error = zsBeginStereoBufferFrame(bufferHandle_);

	// Set the application window's rendering context as the current rendering context.
	//wglMakeCurrent(g_hDC, g_hRC);
	std::unique_lock<std::mutex> cloudLock(remoteCloudMutex_);

	// Draw the scene for each eye.
	drawSceneForEye(ZS_EYE_LEFT);
	drawSceneForEye(ZS_EYE_RIGHT);

	haveNewRemoteCloud_.store(false);
	cloudLock.unlock();

	// Flush the render buffers.
	glutSwapBuffers();
}

#endif