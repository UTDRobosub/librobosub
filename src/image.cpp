#include "cvlib/image.h"

namespace cvlib {
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
	void ImageTransform::scale(Mat& image, Size approxSize)
	{
		Size imageSize = image.size();
		double ratioWidth = (double)approxSize.width / (double)imageSize.width;
		double ratioHeight = (double)approxSize.height / (double)imageSize.height;
		double ratio = std::min(ratioWidth, ratioHeight);
		scale(image, ratio);
	}

	void Drawing::text(Mat& img, String text, Point origin, Scalar color, Anchor anchor, float scale) {
		if (anchor == Anchor::BOTTOM_LEFT)
			ImageTransform::flip(img, ImageTransform::FlipAxis::HORIZONTAL);
		putText(img, text, origin, cv::FONT_HERSHEY_SIMPLEX, scale, color, 2, cv::LINE_AA,
			(anchor == Anchor::BOTTOM_LEFT || anchor == Anchor::BOTTOM_LEFT_UNFLIPPED_Y));
		if (anchor == Anchor::BOTTOM_LEFT)
			ImageTransform::flip(img, ImageTransform::FlipAxis::HORIZONTAL);
	}
}