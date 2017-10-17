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

		Size size = Size(newWidth, newHeight);
		cv::warpAffine(image, image, rotMatrix, image.size());
	}

	void ImageTransform::scale(Mat& image, double factor)
	{
		resize(image, Size(image.cols * factor, image.rows * factor));
	}
	void ImageTransform::scale(Mat& image, Size approxSize)
	{
		Size imageSize = image.size();
		double ratioWidth = approxSize.width / imageSize.width;
		double ratioHeight = approxSize.height / imageSize.height;
		double ratio = std::min(ratioWidth, ratioHeight);
		scale(image, ratio);
	}
}