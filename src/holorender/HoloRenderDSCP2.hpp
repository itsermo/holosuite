#pragma once

#include "IHoloRender.hpp"
//need this for GL.h macro definitions
#ifdef WIN32
#include <Windows.h>
#undef near
#undef far
#endif

#include <gl/GL.h>
#include <gl/GLU.h>
#include <GL/glut.h>

#include <Cg/cg.h>
#include <Cg/cgGL.h>

#define HOLO_RENDER_DSCP2_NUM_DEBUG_SWITCHES 10
#define HOLO_RENDER_DSCP2_NUM_MONITOR_HEADS 3
#define HOLO_RENDER_DSCP2_PROGRAM_NAME = "edu.mit.media.obmg.dscp.mk2"

#include <atomic>
#include <string>

namespace holo
{
	namespace render
	{
		class HoloRenderDSCP2 : IHoloRender
		{
		public:
			HoloRenderDSCP2();
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

		private:

			static HoloRenderDSCP2 *gCurrentInstance;

			// GL and GLUT related functions
			static void glutDisplay(void);
			static void glutIdle(void);
			static void glutKeyboard(unsigned char c, int x, int y);
			static void glutCleanup(void);

			void glCheckErrors(void);

			// Drawing functions
			void drawString(float posX, float posY, std::string theString);
			void drawScene(float *eyePosition, float *modelMatrix_sphere,
				float *invModelMatrix_sphere, float *objSpaceLightPosition_sphere,
				float *modelMatrix_cone, float *invModelMatrix_cone,
				float *objSpaceLightPosition_cone, float h, float v, int drawdepthOn,
				float myobject, int viewnumber);

			int headNumber_ = 0;

			float masterHologramGain_;

			int viewEnableBitmask_;  //set bits to zero to skip rendering views 
			int zeroDepth_; //should we set depth-view to zero (for debuging)
			bool zeroModulation_; // " " " color
			float fakeZ_; //pretend all points on object is at this z-depth
			float fakeModulation_; //pretend all points on object is at this z-depth
			int hologramOutputDebugSwitch_; //send different intermediate variables to color buffer when rendering

			GLuint DLid_;

			std::atomic<int> enableDrawDepth_, rotateCounter_; //for enabling rotation
			int frameNumber_, currentTime_, timeBase_; // for counting FPS

			int displayModeWidth_, displayModeHeight_;
			int viewTexWidth_, viewTexHeight_;

			// Render view options
			int numViews_;
			float hologramPlaneWidthMM_, hologramPlaneHeightMM_, nearPlaneMM_m, farPlaneMM_;
			float nearPlane_, farPlane_;
			int numX_, numY_, tileY_, tileX_, numRGB_;

			GLubyte *localFramebufferStore_;

			float mag_, fieldOfView_, hogelSwitch_;

			GLuint *textureID_;
			GLuint meshTexID_;
			GLuint texNum_;

			GLfloat *lightAmbient_;
			GLfloat *lightDiffuse_;
			GLfloat *lightPosition_;

			void checkForCgErrorLine(const char *situation, int line = 0);
			void checkForCgError2(const char *situation);

			void cgSetBrassMaterial(void);
			void cgSetRedPlasticMaterial(void);
			void cgSetEmissiveLightColorOnly(void);

			const char *normalMapLightingProgramName_;
			const char *normalMapLightingProgramFileName_;
			const char *normalMapLightingVertexProgramFileName_;

			CGcontext normalMapLightingCgContext_;
			CGprofile normalMapLightingCgVertexProfile_;
			CGprofile normalMapLightingCgFragmentProfile_;
			CGprogram normalMapLightingCgVertexProgram_, normalMapLightingCgFragmentProgram_;

			CGparameter cgVertexParamModelUIScale_;

			CGparameter cgVertexParamModelViewProj_;
			CGparameter cgFragmentParamGlobalAmbient_,
				cgFragmentParamLightColor_, cgFragmentParamLightPosition_,
				cgFragmentParamEyePosition_, cgFragmentParamKe_,
				cgFragmentParamKa_, cgFragmentParamKd_, cgFragmentParamKs_,
				cgFragmentParamShininess_, cgFragmentParamDrawdepth_, cgFragmentParamHeadnum_;

			CGparameter cgFragmentParamHogelYes_, cgFragmentParamHologramGain_,
				cgFragmentParamHologramDebugSwitch_, cgVertexParamTextureMatrix_,
				cgVertexParamDepthMatrix_, cgVertexParamDrawdepth_;

			CGprofile cgVertexProfile_, cgFragmentProfile_;
			CGprogram cgVertexProgram_, cgFragmentProgram_;
			CGparameter cgFragmentParamDecal0_,
				cgFragmentParamDecal1_, cgFragmentParamDecal2_, cgFragmentParamDecal3_;

			const char *programName_, *vertexProgramFileName_, *vertexProgramName_;
			const char *fragmentProgramFileName_, *fragmentProgramName_;

			GLuint allViewsFBO_;
			GLuint depthBuffer_;

			GLfloat *projectionMatrix1_;
			GLfloat *projectionMatrix2_;

			float *globalAmbient_, *lightColor_;

			std::atomic<float> lightLocationX_, lightLocationY_, lightLocationZ_;
			std::atomic<float> translateX_, translateY_, translateZ_;
			std::atomic<float> rot_, rotX_;

		};
	}
}