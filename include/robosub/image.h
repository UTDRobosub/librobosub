#pragma once

#define _USE_MATH_DEFINES

#include "common.h"
#include "color.h"
#include <opencv2/opencv.hpp>
#include <math.h>

namespace robosub
{
	class ImageFilter 
	{

	};

	class ImageTransform 
	{
	public:
		enum FlipAxis
		{
			HORIZONTAL = 0,
			VERTICAL = 1,
			BOTH = -1
		};

		///Flip image across a given axis
		EXPORT static void flip(Mat& image, FlipAxis axis);
		///Rotate the image by a certain angle (in degrees)
		EXPORT static void rotate(Mat& image, double angle);
		///Scale image by a scaling factor (1.0 = no scale)
		EXPORT static void scale(Mat& image, double factor);
		///Scale image to an approximate size
		EXPORT static void scale(Mat& image, Size approxSize);
		///Resize image arbitrarily
		EXPORT static void resize(Mat& image, Size size);
	};

	class Drawing 
	{
	public:
		enum Anchor 
		{
			TOP_LEFT,
			BOTTOM_LEFT,
			BOTTOM_LEFT_UNFLIPPED_Y
		};

		EXPORT static void text(Mat& img, String text, Point origin, Scalar color = Scalar(255,255,255,255), Anchor anchor = TOP_LEFT, float scale = 1.0);
	};
}