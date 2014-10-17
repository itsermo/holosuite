#pragma once

#include "../holocommon/CommonDefs.hpp"
#include "IHoloRender.hpp"
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

#define HOLO_RENDER_OPENGL_ENABLE_ZSPACE_RENDERING true
#define HOLO_RENDER_OPENGL_DEFAULT_VOXEL_SIZE HOLO_RENDER_DEFAULT_VOXEL_SIZE

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
			int voxelSize_;

			int mouseLeftButton_;
			int mouseMiddleButton_;
			int mouseRightButton_;
			int mouseDownX_;
			int mouseDownY_;

			int windowWidth_, windowHeight_;

			float viewPhi_, viewTheta_, viewDepth_;

			log4cxx::LoggerPtr logger_;
		};

		static HoloRenderOpenGL *gCurrentOpenGLInstance = nullptr;
	}
}