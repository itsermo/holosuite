#pragma once

#include "../holocommon/CommonDefs.hpp"
#include "IHoloRender.hpp"
#include "HoloRenderObjectTracker.hpp"

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/organized_fast_mesh.h>

#ifdef WIN32
#include <Windows.h>
#endif

#ifndef __APPLE__
#include <GL/gl.h>
#include <GL/glu.h>
#else
#include <OpenGL/gl.h>
#endif

#include <GLFW/glfw3.h>

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

			bool getIsFullScreen() { return isFullScreen_; }
			void setFullScreen(bool shouldFullScreen);
			void toggleFullScreen(){ shouldToggleFullScreen_ = true; }

			void setFrameBufferSize(int width, int height) { windowWidth_ = width; windowHeight_ = height; }

		private:

			void display();

			bool initWindow(GLFWwindow** window, bool shouldFullscreen);
			void deinitWindow(GLFWwindow** window);

			static void keyboardEvent(GLFWwindow* window, int key, int scancode, int action, int mods);
			static void frameBufferResizeEvent(GLFWwindow* window, int width, int height);

			void renderLoop();
			void glCheckErrors();

			void drawPointCloud(GLuint glCloudBuffer, HoloCloudPtr & theCloud);
			void drawObjects(bool overrideColor);
			void drawBackgroundGrid(GLfloat width, GLfloat height, GLfloat depth);
			void drawSphere(GLfloat x, GLfloat y, GLfloat z, GLfloat radius);

			GLFWwindow* window_;
			std::atomic<bool> shouldToggleFullScreen_;

			std::mutex localCloudMutex_;
			HoloCloudPtr localCloud_;

			std::mutex remoteCloudMutex_;
			HoloCloudPtr remoteCloud_;

			std::atomic<bool> haveNewRemoteCloud_;
			std::atomic<bool> haveNewLocalCloud_;
			std::thread renderThread_;

			bool isInit_;
			bool firstInit_;

			std::mutex hasInitMutex_;
			std::mutex hasQuitMutex_;
			std::condition_variable hasInitCV_;
			std::condition_variable hasQuitCV_;

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

			GLuint cloudGLBuffer_[2];
			bool haveCloudGLBuffer_;
			bool enableMeshConstruction_;

			pcl::OrganizedFastMesh<HoloPoint3D> organizedFastMesh_;
			boost::shared_ptr<std::vector<pcl::Vertices>> organizedFastMeshVertices_;
			pcl::PolygonMesh::Ptr mesh_;

			std::atomic<bool> shouldRun_;

			log4cxx::LoggerPtr logger_;
		};

		static HoloRenderOpenGL *gCurrentOpenGLInstance = nullptr;
	}
}
