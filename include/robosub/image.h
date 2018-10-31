#pragma once

#define _USE_MATH_DEFINES

#include "common.h"
#include "primitives.h"
#include <opencv2/opencv.hpp>
#include <math.h>

namespace robosub
{
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
		EXPORT static void scale(Mat& image, Size approxSize, bool maximize = false);
		///Resize image arbitrarily
		EXPORT static void resize(Mat& image, Size size);
	};

	class ImageFilter
	{
		public:
		///Equalize image histogram, maximizing contrast of the image
		///This method does not maximize visual contrast - use equalizeHistogramCLAHE instead.
		EXPORT static void equalizeHistogram(Mat& image);
		
		EXPORT static int getBlurCoefficient(Mat& image);
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

		EXPORT static Size getTextSize(String text, double scale, int thickness);

		EXPORT static void transparentImage(Mat* src, Mat* overlay, const Point& location);

		EXPORT static void rectangle(Mat& img, Point one, Point two, Scalar& borderColor, Scalar fillColor = Scalar(), int thickness = 1, int lineType = LINE_8);

		EXPORT static void rectangle(Mat& img, Rect rectangle, Scalar& borderColor, Scalar fillColor = Scalar(), int thickness = 1, int lineType = LINE_8);

		EXPORT static void contours(Mat& img, vector<Contour>& contours, Scalar& borderColor, Scalar fillColor = Scalar(), int thickness = 1, int lineType = LINE_8);

		EXPORT static void text(Mat& img, String text, Point origin, Scalar color = Scalar(255, 255, 255, 255), Anchor anchor = TOP_LEFT, double scale = 1.0, int thickness = 2);
	};
}
