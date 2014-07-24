#pragma once
#include "../common/CommonDefs.hpp"
namespace holo
{
	namespace utils
	{

		//converts rgba + z pictures into a point cloud and reprojects to real-world coordinates quickly
		void ReprojectToRealWorld(HoloCloudPtr& cloudOut, HoloRGBAZMat& rgbaz, holo::capture::WorldConvertCache& worldConvertCache);

		//converts rgb888 to rgba8888, (aka rgb to rgba)
		void ConvertRGBToRGBA(cv::Mat& rgbMat, cv::Mat& rgbaMatOut);

		// Following files were copied from 
		/*
		*  transforms.h
		*  Dblfrustum
		*
		*  Created by HoloVideo Bove Lab on 6/11/08.
		*  Copyright 2008 __MyCompanyName__. All rights reserved.
		*
		*/
		void BuildShearOrthographicMatrix(double Xleft, double Xright, double Ybot, double Ytop,
			double n, double f, double q,
			float m[16]);

		void BuildShearOrthographicMatrix2(double Xleft, double Xright, double Ybot, double Ytop,
			double n, double f, double q,
			float m[16]);

		void BuildDblPerspectiveMatrix(double Xleft, double Xright, double Ybot, double Ytop,
			double n, double f, double p,
			float m[16]);


		void BuildDblPerspectiveMatrix2(double Xleft, double Xright, double Ybot, double Ytop,
			double n, double f, double p,
			float m[16]);


		void BuildPerspectiveMatrix(double fieldOfView,
			double aspectRatio,
			double zNear, double zFar,
			float m[16]);

		/* Build a row-major (C-style) 4x4 matrix transform based on the
		parameters for gluLookAt. */
		void BuildLookAtMatrix(double eyex, double eyey, double eyez,
			double centerx, double centery, double centerz,
			double upx, double upy, double upz,
			float m[16]);

		/* Build a row-major (C-style) 4x4 matrix transform based on the
		parameters for glRotatef. */
		void MakeRotateMatrix(float angle,
			float ax, float ay, float az,
			float m[16]);

		/* Build a row-major (C-style) 4x4 matrix transform based on the
		parameters for glTranslatef. */
		void MakeTranslateMatrix(float x, float y, float z, float m[16]);

		/* Build a row-major (C-style) 4x4 matrix transform based on the
		parameters for glTranslatef. */
		void MakeScaleMatrix(float x, float y, float z, float m[16]);


		/* Simple 4x4 matrix by 4x4 matrix multiply. */
		void MultMatrix(float dst[16],
			const float src1[16], const float src2[16]);

		/*Invert a row-major (C-style) 4x4 model (trans,rot) matrix */
		void TransposeMatrix(float *out, const float *m);

		/* Invert a row-major (C-style) 4x4 matrix. */
		void InvertMatrix(float *out, const float *m);

		/* Simple 4x4 matrix by 4-component column vector multiply. */
		void Transform(float dst[4],
			const float mat[16], const float vec[4]);
	}
}