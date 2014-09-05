#pragma once
#ifdef ENABLE_HOLO_DSCP2
#include "IHoloRender.hpp"

#include <log4cxx/log4cxx.h>
//need this for GL.h macro definitions
#ifdef WIN32
#include <Windows.h>
#endif

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#include <thread>
#include <mutex>

#define HOLO_RENDER_DSCP2_NUM_DEBUG_SWITCHES 10
#define HOLO_RENDER_DSCP2_NUM_MONITOR_HEADS 3
#define HOLO_RENDER_DSCP2_MASTER_HOLOGRAM_GAIN 1.0f
#define HOLO_RENDER_DSCP2_VIEW_ENABLE_BITMASK 0xffffffff
#define HOLO_RENDER_DSCP2_ZERO_DEPTH 0
#define HOLO_RENDER_DSCP2_ZERO_MODULATION 0
#define HOLO_RENDER_DSCP2_FAKE_Z 0.5f
#define HOLO_RENDER_DSCP2_FAKE_MODULATION 0.5f
#define HOLO_RENDER_DSCP2_DISPLAY_MODE_WIDTH 2045
#define HOLO_RENDER_DSCP2_DISPLAY_MODE_HEIGHT 3514
#define HOLO_RENDER_DSCP2_VIEW_RENDER_RES_HORIZ 512
#define HOLO_RENDER_DSCP2_VIEW_RENDER_RES_VERT 144
#define HOLO_RENDER_DSCP2_VIEW_RENDER_TILE_X 2
#define HOLO_RENDER_DSCP2_VIEW_RENDER_TILE_Y 2
#define HOLO_RENDER_DSCP2_VIEW_RENDER_NUM_VIEWS_PER_PIXEL 4
#define HOLO_RENDER_DSCP2_VIEW_RENDER_MAG 1.0f
#define HOLO_RENDER_DSCP2_VIEW_RENDER_FOV 30.0f
#define HOLO_RENDER_DSCP2_VIEW_RENDER_HOLOGRAM_PLANE_WIDTH_MM 2.75f
#define HOLO_RENDER_DSCP2_VIEW_RENDER_HOLOGRAM_PLANE_HEIGHT_MM 1.35f
#define HOLO_RENDER_DSCP2_PLANE_NEAR 0.01f
#define HOLO_RENDER_DSCP2_PLANE_FAR 3.0f
#define HOLO_RENDER_DSCP2_LIGHT_LOCATION_X 0.0f
#define HOLO_RENDER_DSCP2_LIGHT_LOCATION_Y -7.900f
#define HOLO_RENDER_DSCP2_LIGHT_LOCATION_Z 1.100f

// CG code defines
#define HOLO_RENDER_DSCP2_WINDOW_NAME "edu.mit.media.obmg.dscp.mk2"

#define HOLO_RENDER_DSCP2_CG_PROGRAM_PATH "../shaders"

#define HOLO_RENDER_DSCP2_CG_PARALLAX_VIEW_VERTEX_PROGRAM_NAME "HoloRenderDSCP2CloudVertexShader"
#define HOLO_RENDER_DSCP2_CG_PARALLAX_VIEW_VERTEX_PROGRAM_FILENAME "HoloRenderDSCP2CloudVertexShader.cg"
#define HOLO_RENDER_DSCP2_CG_PARALLAX_VIEW_FRAGMENT_PROGRAM_NAME "HoloRenderDSCP2CloudFragmentShader"
#define HOLO_RENDER_DSCP2_CG_PARALLAX_VIEW_FRAGMENT_PROGRAM_FILENAME "HoloRenderDSCP2CloudFragmentShader.cg"

#define HOLO_RENDER_DSCP2_CG_FRINGE_PATTERN_VERTEX_PROGRAM_NAME "HoloRenderDSCP2FringeVertexShader"
#define HOLO_RENDER_DSCP2_CG_FRINGE_PATTERN_VERTEX_PROGRAM_FILENAME "HoloRenderDSCP2FringeVertexShader.cg"
#define HOLO_RENDER_DSCP2_CG_FRINGE_PATTERN_FRAGMENT_PROGRAM_NAME "HoloRenderDSCP2FringeFragmentShader"
#define HOLO_RENDER_DSCP2_CG_FRINGE_PATTERN_FRAGMENT_PROGRAM_FILENAME "HoloRenderDSCP2FringeFragmentShader.cg"

#include <atomic>
#include <string>
#include <condition_variable>

namespace holo
{
	namespace render
	{
		class HoloRenderDSCP2 : public IHoloRender
		{
		public:
			HoloRenderDSCP2();
			HoloRenderDSCP2(int headNumber);
			~HoloRenderDSCP2();

			bool init();
			void deinit();
			//virtual void updateFromMats(cv::Mat rgbaImage, cv::Mat depthImage) = 0;
			void updateFromPointCloud(HoloCloudPtr && pointCloud);
			void* getContext() { return nullptr; }

			void display(void);
			void idle(void);
			void keyboard(unsigned char c, int x, int y);
			void cleanup(void);

			struct ParallaxViewVertexArgs
			{
				CGparameter modelViewProj;
				CGparameter modelUIScale;
			};

			struct ParallaxViewFragmentArgs
			{
				CGparameter globalAmbient;
				CGparameter lightColor;
				CGparameter lightPosition;
				CGparameter eyePosition;
				CGparameter ke;
				CGparameter ka;
				CGparameter kd;
				CGparameter ks;
				CGparameter	shininess;
				CGparameter drawDepth;
				CGparameter headNum;
				CGparameter decal;
			};

			struct FringePatternVertexArgs
			{
				CGparameter placeHolder;
			};

			struct FringePatternFragmentArgs
			{
				CGparameter hogelYes;
				CGparameter hologramGain;
				CGparameter debugSwitch;
				CGparameter textureMatrix;
				CGparameter depthMatrix;
				CGparameter decal;
				CGparameter headNum;
			};

		private:

			// GL and GLUT related functions
			static void glutDisplay();
			static void glutIdle();
			static void glutKeyboard(unsigned char c, int x, int y);
			static void glutCleanup();
			void glutInitLoop();

			void glCheckErrors();

			// Drawing functions
			void drawString(float posX, float posY, std::string theString);
			void drawScene(float *eyePosition, float *modelMatrix_sphere,
				float *invModelMatrix_sphere, float *objSpaceLightPosition_sphere,
				float *modelMatrix_cone, float *invModelMatrix_cone,
				float *objSpaceLightPosition_cone, float h, float v, int drawdepthOn,
				float myobject, int viewnumber);

			void drawPointCloud();

			//cg error checking, return true if Cg error
			bool checkForCgErrorLine(const char *situation, int line = 0);
			bool checkForCgError2(const char *situation);
			bool checkForCgError(const char *situation);

			void cgSetBrassMaterial();
			void cgSetRedPlasticMaterial();
			void cgSetEmissiveLightColorOnly();

			int headNum_;

			float masterHologramGain_;

			int viewEnableBitmask_;  //set bits to zero to skip rendering views 
			int zeroDepth_; //should we set depth-view to zero (for debuging)
			int zeroModulation_; // " " " color
			float fakeZ_; //pretend all points on object is at this z-depth
			float fakeModulation_; //pretend all points on object is at this z-depth
			int hologramOutputDebugSwitch_; //send different intermediate variables to color buffer when rendering

			//GLuint DLid_;

			int enableDrawDepth_, rotateCounter_; //for enabling rotation
			int frameNumber_, currentTime_, timeBase_; // for counting FPS

			int displayModeWidth_, displayModeHeight_;
			int viewTexWidth_, viewTexHeight_;

			// Render view options
			int numViews_;
			float hologramPlaneWidthMM_, hologramPlaneHeightMM_;
			float nearPlane_, farPlane_;
			int numX_, numY_, tileY_, tileX_, numViewsPerPixel_;

			GLubyte *localFramebufferStore_;

			float mag_, fieldOfView_;

			GLuint textureID_;

			GLfloat lightAmbient_[4];
			GLfloat lightDiffuse_[4];
			GLfloat lightPosition_[4];
			GLfloat lightColor_[3];
			GLfloat globalAmbient_[3];

			GLfloat projectionMatrix1_[16];
			GLfloat projectionMatrix2_[16];

			float lightLocationX_, lightLocationY_, lightLocationZ_;
			float translateX_, translateY_, translateZ_;
			float rot_, rotX_;

			const char *parallaxViewVertexProgramName_, *parallaxViewVertexProgramFileName_;
			const char *parallaxViewFragmentProgramName_, *parallaxViewFragmentProgramFileName_;

			const char *fringePatternVertexProgramFileName_, *fringePatternVertexProgramName_;
			const char *fringePatternFragmentProgramFileName_, *fringePatternFragmentProgramName_;

			CGcontext cgContext_;

			CGprogram parallaxViewCgVertexProgram_, parallaxViewCgFragmentProgram_;
			CGprofile parallaxViewCgVertexProfile_, parallaxViewCgFragmentProfile_;
			ParallaxViewVertexArgs parallaxViewVertexArgs_;
			ParallaxViewFragmentArgs parallaxViewFragmentArgs_;

			CGprogram fringePatternCgVertexProgram_, fringePatternCgFragmentProgram_;
			CGprofile fringePatternCgVertexProfile_, fringePatternCgFragmentProfile_;
			FringePatternFragmentArgs fringeFragmentArgs_;

			std::mutex cloudMutex_;
			HoloCloudPtr cloud_;
			std::atomic<bool> haveNewCloud_;
			std::thread glutInitThread_;

			bool isInit_;
			bool firstInit_;

			std::mutex hasInitMutex_;
			std::condition_variable hasInitCV_;

			//cv::Mat localFrameImg_;

			log4cxx::LoggerPtr logger_;
		};

		static HoloRenderDSCP2 *gCurrentDSCP2Instance = nullptr;
	}
}

#endif