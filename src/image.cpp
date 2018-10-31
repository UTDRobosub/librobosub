#include "robosub/image.h"

namespace robosub {
	void ImageTransform::resize(Mat& image, Size size)
	{
		int interpolation;
		if (size.width == image.size().width && size.height == image.size().height)
			return;
		else if (size.width > image.size().width && size.height > image.size().height)
			interpolation = cv::INTER_CUBIC; //enlarge image
		else if (size.width < image.size().width && size.height < image.size().height)
			interpolation = cv::INTER_AREA; //shrink image
		else
			interpolation = cv::INTER_LINEAR; //not entirely sure, so use safe option
		cv::resize(image, image, size, 0, 0, interpolation);
	}

	void ImageTransform::flip(Mat& image, FlipAxis axis)
	{
		cv::flip(image, image, axis);
	}

	void ImageTransform::rotate(Mat& image, double angle)
	{
		double radians = angle / 180.0 * M_PI;
		double sin = std::abs(std::sin(radians));
		double cos = std::abs(std::cos(radians));

		double newWidth = (image.cols * cos + image.rows * sin);
		double newHeight = (image.cols * sin + image.rows * cos);

		Point center = Point((int)(newWidth / 2.0), (int)(newHeight / 2.0));
		Mat rotMatrix =  cv::getRotationMatrix2D(center, angle, 1.0);

		Size size = Size((int)newWidth, (int)newHeight);
		cv::warpAffine(image, image, rotMatrix, image.size());
	}

	void ImageTransform::scale(Mat& image, double factor)
	{
		resize(image, Size((int)(image.cols * factor), (int)(image.rows * factor)));
	}

	void ImageTransform::scale(Mat& image, Size approxSize, bool maximize)
	{
		Size imageSize = image.size();
		double ratioWidth = (double)approxSize.width / (double)imageSize.width;
		double ratioHeight = (double)approxSize.height / (double)imageSize.height;
		double ratio;
		if (maximize) ratio = std::max(ratioWidth, ratioHeight);
		else ratio = std::min(ratioWidth, ratioHeight);
		scale(image, ratio);
	}

	Size Drawing::getTextSize(String text, double scale, int thickness)
	{
		return cv::getTextSize(text, FONT_HERSHEY_COMPLEX, scale, thickness, 0);
	}

	void Drawing::transparentImage(Mat * src, Mat * overlay, const Point & location)
	{
		for (int y = max(location.y, 0); y < src->rows; ++y)
		{
			int fY = y - location.y;

			if (fY >= overlay->rows)
				break;

			for (int x = max(location.x, 0); x < src->cols; ++x)
			{
				int fX = x - location.x;

				if (fX >= overlay->cols)
					break;

				double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3]) / 255;

				for (int c = 0; opacity > 0 && c < src->channels(); ++c)
				{
					unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
					unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
					src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
				}
			}
		}
	}

	void Drawing::rectangle(Mat & img, Point one, Point two, Scalar & borderColor, Scalar fillColor, int thickness, int lineType)
	{
		int left = min(one.x, two.x);
		int right = max(one.x, two.x);
		int top = min(one.y, two.y);
		int bottom = max(one.y, two.y);

		Rect rect = Rect(left, top, right - left, bottom - top);

		rectangle(img, rect, borderColor, fillColor, thickness, lineType);
	}

	void Drawing::rectangle(Mat & img, Rect rectangle, Scalar & borderColor, Scalar fillColor, int thickness, int lineType)
	{
		vector<Point> points;
		points.push_back(Point(rectangle.x, rectangle.y));
		points.push_back(Point(rectangle.x + rectangle.width, rectangle.y));
		points.push_back(Point(rectangle.x + rectangle.width, rectangle.y + rectangle.height));
		points.push_back(Point(rectangle.x, rectangle.y + rectangle.height));

		vector<Contour> contours;
		contours.push_back(Contour(points));

		Drawing::contours(img, contours, borderColor, fillColor, thickness, lineType);
	}

	void Drawing::contours(Mat& img, vector<Contour>& contours, Scalar & borderColor, Scalar fillColor, int thickness, int lineType)
	{
		vector<Mat> _contours;
		for (Contour& contour : contours) {
			_contours.push_back(contour.getMat());
		}

		//draw border
		drawContours(img, _contours, -1, borderColor, thickness, lineType);
		//fill
		if (fillColor != Scalar()) {
			for (Contour& contour : contours) {
				fillConvexPoly(img, contour.getPoints(), fillColor);
			}
		}
	}

	void Drawing::text(Mat& img, String text, Point origin, Scalar color, Anchor anchor, double scale, int thickness) {
		if (anchor == Anchor::BOTTOM_LEFT)
			ImageTransform::flip(img, ImageTransform::FlipAxis::HORIZONTAL);
		putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv::LINE_AA,
			(anchor == Anchor::BOTTOM_LEFT || anchor == Anchor::BOTTOM_LEFT_UNFLIPPED_Y));
		if (anchor == Anchor::BOTTOM_LEFT)
			ImageTransform::flip(img, ImageTransform::FlipAxis::HORIZONTAL);
	}

	void ImageFilter::equalizeHistogram(Mat& image)
	{
		equalizeHist(image, image);
	}
	
	int ImageFilter::getBlurCoefficient(Mat& image){
		assert(image.channels()==1);
		
		Mat laplace;
		Laplacian(image, laplace, CV_8U, 3, 1, 0, BORDER_DEFAULT);
		
		Scalar mean, stddev;
		meanStdDev(laplace, mean, stddev, Mat());
		
		return (int)(stddev[0]*stddev[0]);
	}
}
