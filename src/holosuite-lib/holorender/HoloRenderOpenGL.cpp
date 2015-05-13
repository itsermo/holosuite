#include "HoloRenderOpenGL.hpp"

using namespace holo;
using namespace holo::render;

std::atomic<bool> gShouldRun_;

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
	windowX_(0),
	windowY_(0),
	windowWidth_(1024),
	windowHeight_(768),
	viewPhi_(0.0f),
	viewTheta_(0.0f),
	viewDepth_(0.2f),
	isFullScreen_(false),
	prevWindowWidth_(0),
	prevWindowHeight_(0),
	prevWindowX_(0),
	prevWindowY_(0),
	haveCloudGLBuffer_(false),
	enableMeshConstruction_(false)
{
	logger_ = log4cxx::Logger::getLogger("edu.mit.media.obmg.holosuite.render.opengl");
	LOG4CXX_DEBUG(logger_, "Instantiating HoloRenderOpenGL object...");
	gCurrentOpenGLInstance = this;
	LOG4CXX_DEBUG(logger_, "Done instantiating HoloRenderOpenGL object");

	cloudGLBuffer_[0] = 0;
	cloudGLBuffer_[1] = 0;
}

HoloRenderOpenGL::~HoloRenderOpenGL()
{
	LOG4CXX_DEBUG(logger_, "Destroying HoloRenderOpenGL object...");
	deinit();
	LOG4CXX_DEBUG(logger_, "Done destroying HoloRenderOpenGL object");
	gCurrentOpenGLInstance = nullptr;
}

bool HoloRenderOpenGL::init()
{
	LOG4CXX_INFO(logger_, "Initializing OpenGL render algorithm...");

	localCloud_ = HoloCloudPtr(new HoloCloud);
	remoteCloud_ = HoloCloudPtr(new HoloCloud);

	if (enableMeshConstruction_)
	{
		organizedFastMesh_.setTrianglePixelSize(1);
		organizedFastMesh_.setTriangulationType(pcl::OrganizedFastMesh<HoloPoint3D>::TRIANGLE_ADAPTIVE_CUT);
	}

	gShouldRun_ = true;
	glutInitThread_ = std::thread(&HoloRenderOpenGL::glutInitLoop, this);

	std::unique_lock<std::mutex> lg(hasInitMutex_);
	hasInitCV_.wait(lg);



	return isInit_;
}

void HoloRenderOpenGL::deinit()
{
	if (isInit_)
	{
		glutLeaveMainLoop();
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		glutInitThread_.join();
	}
}


void HoloRenderOpenGL::glutInitLoop()
{
#ifdef ENABLE_HOLO_ZSPACE
	if (this->enableZSpaceRendering_)
	{
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
	}
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
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitContextVersion(3, 1);
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

	GLfloat lightAmbientColor[] = { 0.6, 0.3, 0.3, 1 };
	GLfloat lightDiffuseColor[] = { 0.6, 1, 1, 1 };
	GLfloat lightSpecularColor[] = { 0.2, 0.2, 0.2, 1 };
	GLfloat lightGlobalAmbient[] = { 0.2, 0.2, 0.2, 1 };

	GLfloat materialSpecular[] = { 1.0, 1.0, 1.0, 1.0 };
	GLfloat materialShininess[] = { 50.0 };

	// GLUT settings
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f); // Black Background

	glShadeModel(GL_SMOOTH); // Enable Smooth Shading
	glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbientColor);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuseColor);
	glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecularColor);

	//glLightModelfv(GL_AMBIENT, lightGlobalAmbient);

	glMaterialfv(GL_FRONT, GL_SPECULAR, materialSpecular);
	glMaterialfv(GL_FRONT, GL_SHININESS, materialShininess);

	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);

	//Takes care of occlusions for point cloud
	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0.0f, 0.345f, 0.222f,   // Eye
	0.0f, 0.0f, 1.0f,     // Center
	0.0f, 1.0f, 1.0f);    // Up

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//glDisable(GL_LIGHTING);

	glCheckErrors();

#ifndef _DEBUG
	prevWindowWidth_ = glutGet(GLUT_WINDOW_WIDTH);
	prevWindowHeight_ = glutGet(GLUT_WINDOW_HEIGHT);
	prevWindowX_ = glutGet(GLUT_WINDOW_X);
	prevWindowY_ = glutGet(GLUT_WINDOW_Y);
	glutFullScreen();
	isFullScreen_ = true;
#endif

	glewInit();

	if (enableMeshConstruction_)
	{
		mesh_ = pcl::PolygonMeshPtr(new pcl::PolygonMesh());
		organizedFastMeshVertices_ = boost::shared_ptr<std::vector<pcl::Vertices>>(new std::vector<pcl::Vertices>);
	}

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);

	glGenBuffers(2, cloudGLBuffer_);

	glutMainLoop();

#ifdef ENABLE_HOLO_ZSPACE
	if (enableZSpaceRendering_)
	{
		zsDestroyViewport(viewportHandle_);
		zsDestroyStereoBuffer(bufferHandle_);
		zsShutdown(zSpaceContext_);
	}
#endif

	glDeleteBuffers(2, cloudGLBuffer_);

	hasQuitCV_.notify_all();
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
	if (gShouldRun_)
		glutPostRedisplay();
	else
		glutLeaveMainLoop();
}

void HoloRenderOpenGL::keyboard(unsigned char c, int x, int y)
{
	//printf("keyboard \n");
	switch (c)
	{
	case 'w':
		viewDepth_ += 0.05;
		break;
	case 's':
		viewDepth_ -= 0.05;
		break;
	case 'f':
		if (!isFullScreen_)
		{
			prevWindowWidth_ = glutGet(GLUT_WINDOW_WIDTH);
			prevWindowHeight_ = glutGet(GLUT_WINDOW_HEIGHT);
			prevWindowX_ = glutGet(GLUT_WINDOW_X);
			prevWindowY_ = glutGet(GLUT_WINDOW_Y);
			glutFullScreen();
			isFullScreen_ = true;
		}
		else
		{
			glutPositionWindow(prevWindowX_, prevWindowY_);
			glutReshapeWindow(prevWindowWidth_, prevWindowHeight_);
			isFullScreen_ = false;
		}
		break;
	case '1':
		glEnable(GL_LIGHTING);
		break;
	case '2':
		glDisable(GL_LIGHTING);
		break;

	case 27: /* Esc key */
		glutLeaveMainLoop();
		//gShouldRun_ = false;
		break;
	}

	//int mods = glutGetModifiers();
	//if (mods != 0)
	//{

	//}

	glutPostRedisplay();
}

void HoloRenderOpenGL::display()
{
	if (gShouldRun_)
	{
		if (firstInit_)
		{
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

			//float distance = 0.46f;
			//float camX = distance * -sinf(viewPhi_*(M_PI / 180)) * cosf((viewTheta_)*(M_PI / 180));
			//float camY = distance * -sinf((viewTheta_)*(M_PI / 180)) + 0.3;
			//float camZ = -distance * cosf((viewPhi_)*(M_PI / 180)) * cosf((viewTheta_)*(M_PI / 180));

			glViewport(0, 0, (GLsizei)windowWidth_, (GLsizei)windowHeight_);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(45.0f, static_cast<float>(windowWidth_) / static_cast<float>(windowHeight_), 0.001f, 10.0f);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			gluLookAt(0, 0.22, 0.16f,   // Eye
				0.0f, 0.0f, 1.0f,     // Center
				0.0f, 1.0f, 1.0f);

			glTranslatef(0.0, 0.0, viewDepth_);

			glRotatef(-viewTheta_, 1.0, 0.0, 0.0);
			glRotatef(-viewPhi_, 0.0, 1.0, 0.0);

			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			this->drawBackgroundGrid(4, 2, 3);

			std::unique_lock<std::mutex> cloudLock(remoteCloudMutex_);

			glPushMatrix();
			glTranslatef(-.1f, 0.05, -0.09);

			this->drawPointCloud(cloudGLBuffer_[1], remoteCloud_);

			glPopMatrix();

			haveNewRemoteCloud_.store(false);
			cloudLock.unlock();

			glEnable(GL_LIGHTING);
			glEnable(GL_LIGHT0);
			glEnable(GL_NORMALIZE);

			glTranslatef(0.0, .15f, 0.37);

			this->drawObjects();

			glDisable(GL_LIGHTING);
			glDisable(GL_LIGHT0);
			glDisable(GL_NORMALIZE);


			glViewport(windowWidth_/32, windowHeight_/32, (GLsizei)windowWidth_/8, (GLsizei)windowHeight_/8);
			glMatrixMode(GL_PROJECTION);
			glLoadIdentity();
			gluPerspective(45.0f, static_cast<float>(windowWidth_) / static_cast<float>(windowHeight_), 0.001f, 10.0f);
			glMatrixMode(GL_MODELVIEW);
			glLoadIdentity();
			gluLookAt(0.f, 0.f, 0.f,   // Eye
				0.0f, 0.0f, 1.0f,     // Center
				0.0f, 1.0f, 1.0f);

			//glScalef(0.001f, 0.001f, 0.001f);
			//glTranslatef(-.1f, 0.05, viewDepth_-.3f);
			//glTranslatef(camX+0.2, camY + 0.02f, camZ);
			//glRotatef(180.f, 0, 1, 0);
			std::unique_lock<std::mutex> localCloudLock(localCloudMutex_);

			glTranslatef(0.f, 0.f, 0.6f);
			glScalef(-1.f, 1.f, 1.f);
			glRotatef(-viewPhi_, 0.0, 1.0, 0.0);

			glDisable(GL_DEPTH_TEST);
			this->drawPointCloud(cloudGLBuffer_[0], localCloud_);
			glEnable(GL_DEPTH_TEST);

			haveNewLocalCloud_.store(false);
			localCloudLock.unlock();

			glFinish();

			glutSwapBuffers();

#ifdef ENABLE_HOLO_ZSPACE
		}
#endif
	}
	else
		glutLeaveMainLoop();
	//std::this_thread::sleep_for(std::chrono::milliseconds(13));
}

void HoloRenderOpenGL::cleanup()
{
	//deinit();
}

void HoloRenderOpenGL::mouse(int button, int state, int x, int y)
{
	mouseDownX_ = x; mouseDownY_ = y;

	if (button == 3)
	{
		if (state == GLUT_UP)
			viewDepth_ += 0.05;
	}
	else if (button == 4)
	{
		if (state == GLUT_UP)
			viewDepth_ -= 0.05;
	}
	else
	{
		mouseLeftButton_ = ((button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN));
		mouseMiddleButton_ = ((button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN));
		glutPostRedisplay();
	}
}

void HoloRenderOpenGL::mouseMotion(int x, int y)
{
	if (mouseLeftButton_){ viewPhi_ -= (float)(x - mouseDownX_) / 10.0; viewTheta_ += (float)(mouseDownY_ - y) / 10.0; } // rotate
	if (mouseMiddleButton_){ viewDepth_ += (float)(mouseDownY_ - y) / 100.0; } // scale
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
	std::lock_guard<std::mutex> lg(localCloudMutex_);
	localCloud_ = std::move(pointCloud);
	haveNewLocalCloud_.store(true);
}

void HoloRenderOpenGL::updateRemotePointCloud(HoloCloudPtr && pointCloud)
{
	std::lock_guard<std::mutex> lg(remoteCloudMutex_);
	remoteCloud_ = std::move(pointCloud);
	haveNewRemoteCloud_.store(true);
}
void HoloRenderOpenGL::drawBackgroundGrid(GLfloat width, GLfloat height, GLfloat depth)
{
	const GLfloat lineColor[3] = { 0.2f, 0.2f, 0.2f };
	const GLfloat tileColor[3] = { 0.15f, 0.15f, 0.15f };
	const GLint lineThickness = 10;

	width /= 2;
	height /= 2;

	glColor3f(tileColor[0], tileColor[1], tileColor[2]);
	glBegin(GL_QUADS);

	// bottom plane
	glVertex3f(-width, -height - 0.01, 0);
	glVertex3f(-width, -height - 0.01, depth);
	glVertex3f(width, -height - 0.01, depth);
	glVertex3f(width, -height -0.01, 0);
	//glEnd();

	// far plane
	glVertex3f(-width, -height, depth + 0.01);
	glVertex3f(-width, height, depth + 0.01);
	glVertex3f(width, height, depth + 0.01);
	glVertex3f(width, -height, depth + 0.01);
	//glEnd();

	// top plane
	glVertex3f(-width, height + 0.01, depth);
	glVertex3f(-width, height + 0.01, 0);
	glVertex3f(width, height + 0.01, 0);
	glVertex3f(width, height + 0.01, depth);
	//glEnd();

	// left plane
	glVertex3f(-width - 0.01, -height, 0);
	glVertex3f(-width - 0.01, -height, depth);
	glVertex3f(-width - 0.01, height, depth);
	glVertex3f(-width - 0.01, height, 0);
	//glEnd();

	// right plane
	glVertex3f(width + 0.01, -height, 0);
	glVertex3f(width + 0.01, -height, depth);
	glVertex3f(width + 0.01, height, depth);
	glVertex3f(width + 0.01, height, 0);
	
	glEnd();

	glEnable(GL_LINE_SMOOTH);
	glLineWidth(lineThickness);

	glBegin(GL_LINES);

	glColor3f(lineColor[0], lineColor[1], lineColor[2]);

	for (int i = -height; i <= height; i++)
	{
		glVertex3f(-width, i, depth);
		glVertex3f(width, i, depth);

		glVertex3f(-width, i, 0);
		glVertex3f(-width, i, depth);

		glVertex3f(width, i, 0);
		glVertex3f(width, i, depth);
	}

	for (int i = 0; i <= depth; i++)
	{
		glVertex3f(-width, height, i);
		glVertex3f(width, height, i);
		glVertex3f(-width, -height, i);
		glVertex3f(width, -height, i);

		glVertex3f(-width, -height, i);
		glVertex3f(-width, height, i);

		glVertex3f(width, -height, i);
		glVertex3f(width, height, i);
	}

	for (int i = -width; i <= width; i++) {
		glVertex3f(i, height, 0);
		glVertex3f(i, height, depth);
		glVertex3f(i, -height, 0);
		glVertex3f(i, -height, depth);
		glVertex3f(i, height, depth);
		glVertex3f(i, -height, depth);
	};

	glEnd();
	glDisable(GL_LINE_SMOOTH);
}

void HoloRenderOpenGL::drawPointCloud(GLuint cloudGLBuffer, HoloCloudPtr & theCloud)
{
	if (cloudGLBuffer == 2)
		glDisable(GL_BLEND);

	if (enableMeshConstruction_)
	{
		if (theCloud->size() > 0)
		{
			organizedFastMesh_.setInputCloud(theCloud);
			organizedFastMesh_.reconstruct(*organizedFastMeshVertices_);

			glBegin(GL_TRIANGLES);

			for (int v = 0; v < organizedFastMeshVertices_->size(); v++)
			{
				float x = (*organizedFastMeshVertices_)[v].vertices[0];
				float y = (*organizedFastMeshVertices_)[v].vertices[1];
				float z = (*organizedFastMeshVertices_)[v].vertices[2];

				glColor4f(1.f, 1.f, 1.f, 1.f);
				glVertex3f(x, y, z);
			}

			glEnd();
		}

	}
	else
	{
		glEnable(GL_POINT_SMOOTH);
		glPointSize(cloudGLBuffer == 2 ? voxelSize_ * 4 : 1.f);

		if (cloudGLBuffer > 0)
		{
			glBindBuffer(GL_ARRAY_BUFFER, cloudGLBuffer);
			glBufferData(GL_ARRAY_BUFFER, theCloud->points.size() * sizeof(HoloPoint3D), theCloud->points.data(), GL_STREAM_DRAW);
			glEnableClientState(GL_VERTEX_ARRAY);
			glEnableClientState(GL_COLOR_ARRAY);

			glVertexPointer(3, GL_FLOAT, sizeof(HoloPoint3D), 0);
			glColorPointer(4, GL_UNSIGNED_BYTE, sizeof(HoloPoint3D), (void*)(sizeof(float)* 4));

			glDrawArrays(GL_POINTS, 0, theCloud->points.size());

			glDisableClientState(GL_VERTEX_ARRAY);
			glDisableClientState(GL_COLOR_ARRAY);
			glBindBuffer(GL_ARRAY_BUFFER, 0);
		}
		else
		{
			glBegin(GL_POINTS);

			const float gain = 1 / 255.0; // converting from char units to float
			HoloPoint3D *pointIdx = theCloud->points.data();
			float luma = 0.0f;
			for (int i = 0; i < theCloud->size(); i++)
			{
				if (pointIdx->z == HOLO_CLOUD_BAD_POINT)
					continue;

				glVertex4f(-pointIdx->x, pointIdx->y, pointIdx->z, 1.0f);
				glColor3f(pointIdx->b * gain, pointIdx->g * gain, pointIdx->r * gain);
				pointIdx++;
			}

			glEnd();
		}
		glDisable(GL_POINT_SMOOTH);
	}

	if (cloudGLBuffer == 1)
		glEnable(GL_BLEND);
}

void HoloRenderOpenGL::drawObjects()
{
	if (objectTracker_)
	{
		auto myObjects = objectTracker_->Get3DObjects();
		for (auto obj : myObjects)
		{
			auto info = obj.second->GetObjectInfo();
			auto transform = obj.second->GetTransform();
			auto amOwner = obj.second->GetAmOwner();
			auto isLocal = obj.second->GetIsLocal();

			const float scaleFactor = 1.0f / sqrt(transform.bounding_sphere.w);

			glMatrixMode(GL_MODELVIEW);
			glPushMatrix();

			//if (isLocal)
			//	glTranslatef(-0.15, 0.f, 0.f);
			//else
			//	glTranslatef(0.15, 0.f, 0.f);
			
			if (amOwner)
			{
				glTranslatef(-transform.translate.x, transform.translate.y, -transform.translate.z);
				glRotatef(-transform.rotation.z* 180.f / M_PI, 0, 1, 0);
			}
			else
			{
				glTranslatef(transform.translate.x, transform.translate.y, transform.translate.z);
				glRotatef(transform.rotation.z* 180.f / M_PI, 0, 1, 0);
			}

			glScalef(transform.scale.x, transform.scale.y, transform.scale.z);
			glScalef(isLocal ? scaleFactor*0.1f : -scaleFactor*0.1f, scaleFactor*0.1f, scaleFactor*0.1f);
			if (!isLocal)
				glRotatef(180.f, 0, 1, 0);
			//glRotatef(isLocal ? transform.rotation.z * 180.0f / M_PI : -transform.rotation.z * 180.0f / M_PI, 0, 1, 0);
			glTranslatef(-transform.bounding_sphere.x, -transform.bounding_sphere.y, -transform.bounding_sphere.z);

			if (!obj.second->GetHasGLBuffers())
			{
				GLuint vertBuf = 0;
				GLuint normalBuf = 0;
				GLuint colorBuf = 0;
				glGenBuffers(1, &vertBuf);

				if (obj.second->GetNormalBuffer())
					glGenBuffers(1, &normalBuf);

				if (obj.second->GetColorBuffer())
					glGenBuffers(1, &colorBuf);

				glBindBuffer(GL_ARRAY_BUFFER, vertBuf);
				glBufferData(GL_ARRAY_BUFFER, info.vertex_stride * info.num_vertices, obj.second->GetVertexBuffer(), GL_STREAM_DRAW);
				if (obj.second->GetNormalBuffer())
				{
					glBindBuffer(GL_ARRAY_BUFFER, normalBuf);
					glBufferData(GL_ARRAY_BUFFER, info.vertex_stride * info.num_vertices, obj.second->GetNormalBuffer(), GL_STREAM_DRAW);
				}

				if (obj.second->GetColorBuffer())
				{
					glBindBuffer(GL_ARRAY_BUFFER, colorBuf);
					glBufferData(GL_ARRAY_BUFFER, info.color_stride * info.num_vertices, obj.second->GetColorBuffer(), GL_STREAM_DRAW);
				}

				obj.second->SetGLVertexBufID(vertBuf);
				obj.second->SetGLNormalBufID(normalBuf);
				obj.second->SetGLColorBufID(colorBuf);
				obj.second->SetHasGLBuffers(true);
			}

			if (obj.second->GetHasGLBuffers())
			{
				glEnable(GL_COLOR_MATERIAL);
				glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
				glEnable(GL_NORMALIZE);

				glBindBuffer(GL_ARRAY_BUFFER, obj.second->GetGLVertexBufID());
				glEnableClientState(GL_VERTEX_ARRAY);
				glVertexPointer(info.num_points_per_vertex, GL_FLOAT, info.vertex_stride, 0);

				if (amOwner)
				{
					if (obj.second->GetColorBuffer())
					{
						glBindBuffer(GL_ARRAY_BUFFER, obj.second->GetGLColorBufID());
						glEnableClientState(GL_COLOR_ARRAY);
						glColorPointer(info.num_color_channels, GL_FLOAT, info.color_stride, 0);
					}
					else
						glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
				}
				else
				{
					glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
				}

				if (obj.second->GetNormalBuffer())
				{
					glBindBuffer(GL_ARRAY_BUFFER, obj.second->GetGLNormalBufID());
					glEnableClientState(GL_NORMAL_ARRAY);
					glNormalPointer(GL_FLOAT, info.vertex_stride, 0);
				}

				GLenum faceMode = 0;
				switch (info.num_indecies)
				{
				case 1: faceMode = GL_POINTS; break;
				case 2: faceMode = GL_LINES; break;
				case 3: faceMode = GL_TRIANGLES; break;
				case 4: faceMode = GL_QUADS; break;
				default: faceMode = GL_POLYGON; break;
				}

				glDrawArrays(faceMode, 0, info.num_vertices);
				glDisableClientState(GL_VERTEX_ARRAY);
				glDisableClientState(GL_NORMAL_ARRAY);
				glDisableClientState(GL_COLOR_ARRAY);
				glBindBuffer(GL_ARRAY_BUFFER, 0);

				glDisable(GL_NORMALIZE);
				glDisable(GL_COLOR_MATERIAL);

			}
			else
			{
				glEnable(GL_NORMALIZE);

				auto amOwner = obj.second->GetAmOwner();

				glEnable(GL_COLOR_MATERIAL);
				glBegin(GL_TRIANGLES);

				bool haveNormals = obj.second->GetNormalBuffer() == nullptr ? false : true;
				bool haveColors = obj.second->GetColorBuffer() == nullptr ? false : true;

				float * vp = (float*)obj.second->GetVertexBuffer();
				float * cp = (float*)obj.second->GetColorBuffer();
				float * np = (float*)obj.second->GetNormalBuffer();

				for (int i = 0; i < info.num_vertices; i++, vp+=info.num_indecies, np+=info.num_indecies, cp+=info.num_color_channels)
				{
					glVertex3f(*vp, *(vp+1), *(vp+2));

					if (haveNormals)
						glNormal3f(*np, *(np + 1), *(np + 2));

					if (amOwner && haveColors)
						glColor4f(*cp, *(cp + 1), *(cp + 2), *(cp + 3));
					else if (!haveColors)
						glColor4f(0.5f, 0.5f, 0.5f, 1.0f);
					else
						glColor4f(0.5f, 0.5f, 0.5f, 0.5f);
				}

				glEnd();
				glDisable(GL_NORMALIZE);

				if (amOwner)
					glDisable(GL_COLOR_MATERIAL);

			}

			glPopMatrix();
		}
	}
}

void HoloRenderOpenGL::drawSphere(GLfloat x, GLfloat y, GLfloat z, GLfloat radius)
{
	glPushMatrix();
	glTranslatef(x, y, z);
	glutSolidSphere(radius, 16, 16);
	glPopMatrix();
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
	//GLfloat eyeX = 0.222f * sin(cameraAngle_ * PI / 180.0f);
	//GLfloat eyeY = 0.545f;
	//GLfloat eyeZ = 0.222f * cos(cameraAngle_ * PI / 180.0f);

	// Calculate the camera's new position such that it orbits
	// the world's origin.
	GLfloat eyeX = 0;
	GLfloat eyeY = 0.2;
	GLfloat eyeZ = 0.28f;

	// Use gluLookAt to calculate the new model-view matrix.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eyeX, eyeY, eyeZ,   // Eye
		0.0f, 0.0f, 1.0f,     // Center
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
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	this->drawBackgroundGrid(4, 2, 5);

	glPushMatrix();
	glTranslatef(0.1f, 0.05, -0.09);

	this->drawPointCloud(cloudGLBuffer_[1], remoteCloud_);

	glPopMatrix();

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_NORMALIZE);

	glTranslatef(0.0, 0.30, -0.60);

	this->drawObjects();

	glPushMatrix();
	glTranslatef(-0.15, 0.30, -0.6);

	glutSolidSphere(0.035f, 16, 16);

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);

	glDisable(GL_NORMALIZE);
	glPopMatrix();

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