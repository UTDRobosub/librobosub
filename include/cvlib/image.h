#pragma once

#define _USE_MATH_DEFINES

#include "common.h"
#include <opencv2/opencv.hpp>
#include <math.h>

namespace cvlib
{
	class ImageFilter 
	{

	};

	class ImageTransform 
	{
	private:
		static void resize(Mat& image, Size size);

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
	};

	class Drawing 
	{

	};
}