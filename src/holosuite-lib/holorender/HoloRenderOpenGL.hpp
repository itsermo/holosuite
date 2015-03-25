#pragma once

#include "../holocommon/CommonDefs.hpp"
#include "IHoloRender.hpp"
#include "HoloRenderObjectTracker.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#ifdef WIN32
#include <Windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

#ifdef ENABLE_HOLO_ZSPACE
#include <zSpace.h>
#endif

#define HOLO_RENDER_OPENGL_ENABLE_ZSPACE_RENDERING true
#define HOLO_RENDER_OPENGL_DEFAULT_VOXEL_SIZE HOLO_RENDER_DEFAULT_VOXEL_SIZE

#ifdef ENABLE_HOLO_ZSPACE

#define CHECK_ERROR(error)                                                      \
if (error != ZS_ERROR_OKAY)                                                     \
  {                                                                             \
  char errorString[256];                                                        \
  zsGetErrorString(error, errorString, sizeof(errorString));                    \
  LOG4CXX_ERROR(logger_, errorString);											\
  }

static const float ROTATION_PER_SECOND = 45.0f;
static const float PI = 3.1415926535897932384626433832795f;

#endif

#ifdef ENABLE_HOLO_ODE
#include <ode/ode.h>
#endif
namespace holo
{
	namespace render
	{
		class HoloRenderOpenGL : public IHoloRender
		{
		public:
			HoloRenderOpenGL();
			HoloRenderOpenGL(int voxelSize, bool enableZSpaceRendering);
			~HoloRenderOpenGL();
			bool init();
			void deinit();
			//virtual void updateFromMats(cv::Mat rgbaImage, cv::Mat depthImage) = 0;
			void updateLocalPointCloud(HoloCloudPtr && pointCloud);
			void updateRemotePointCloud(HoloCloudPtr && pointCloud);
			void* getContext();

			void addObjectTracker(boost::shared_ptr<HoloRenderObjectTracker> & objectTracker)
			{
				objectTracker_ = objectTracker;
			}

			void removeObjectTracker()
			{
				objectTracker_.reset();
			}

			void display(void);
			void idle(void);
			void keyboard(unsigned char c, int x, int y);
			void cleanup(void);
			void mouse(int button, int state, int x, int y);
			void mouseMotion(int x, int y);
			void reshape(int width, int height);

		private:

			// GL and GLUT related functions
			static void glutDisplay();
			static void glutIdle();
			static void glutKeyboard(unsigned char c, int x, int y);
			static void glutCleanup();
			static void glutMouse(int button, int state, int x, int y);
			static void glutMouseMotion(int x, int y);
			static void glutReshape(int width, int height);

			void glutInitLoop();
			void glCheckErrors();

			void drawPointCloud();
			void drawObjects();
			void drawBackgroundGrid(GLfloat width, GLfloat height, GLfloat depth);
			void drawSphere(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);

			std::mutex localCloudMutex_;
			HoloCloudPtr localCloud_;

			std::mutex remoteCloudMutex_;
			HoloCloudPtr remoteCloud_;

			std::atomic<bool> haveNewRemoteCloud_;
			std::atomic<bool> haveNewLocalCloud_;
			std::thread glutInitThread_;

			bool isInit_;
			bool firstInit_;

			std::mutex hasInitMutex_;
			std::condition_variable hasInitCV_;

			bool enableZSpaceRendering_;
			float voxelSize_;

			int mouseLeftButton_;
			int mouseMiddleButton_;
			int mouseRightButton_;
			int mouseDownX_;
			int mouseDownY_;

			bool isFullScreen_;
			int windowWidth_, windowHeight_;
			int prevWindowWidth_, prevWindowHeight_;
			int windowX_, windowY_;
			int prevWindowX_, prevWindowY_;

			float viewPhi_, viewTheta_, viewDepth_;

			boost::shared_ptr<HoloRenderObjectTracker> objectTracker_;

#ifdef ENABLE_HOLO_ZSPACE
			ZSContext   zSpaceContext_ = NULL;
			ZSHandle    displayHandle_ = NULL;
			ZSHandle    bufferHandle_ = NULL;
			ZSHandle    viewportHandle_ = NULL;
			ZSHandle    frustumHandle_ = NULL;
			ZSHandle    stylusHandle_ = NULL;

			float       cameraAngle_ = 0.0f;
			ZSMatrix4   cameraMatrix_;
			ZSMatrix4   modelViewMatrix_;
			ZSMatrix4   stylusWorldMatrix_;

			clock_t     previousTime_ = clock();
			bool        isCameraOrbitEnabled_ = false;

			bool update();
			void updateCamera();

			void draw();
			void drawSceneForEye(ZSEye eye);

			void setRenderTarget(ZSEye eye);
			bool setViewMatrix(ZSEye eye);
			bool setProjectionMatrix(ZSEye eye);

			void matrixMultiply(const float a[16], const float b[16], float r[16]);
			void matrixInverse(const float m[16], float i[16]);
#endif

			std::string displayEnv_;

			log4cxx::LoggerPtr logger_;
		};

		static HoloRenderOpenGL *gCurrentOpenGLInstance = nullptr;
	}
}
