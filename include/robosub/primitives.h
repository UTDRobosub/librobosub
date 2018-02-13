#pragma once

#include "common.h"
#include <opencv2/opencv.hpp>

namespace robosub {
	class Detectable {
	protected:
		virtual double left() = 0;
		virtual double right() = 0;
		virtual double top() = 0;
		virtual double bottom() = 0;
	public:
		virtual double width() = 0;
		virtual double height() = 0;
		virtual Mat getMask(Mat& img) = 0;

		inline Point2d topLeft() {
			return Point2d(left(), top());
		}
		inline Point2d bottomRight() {
			return Point2d(right(), bottom());
		}
		inline Scalar averageColor(Mat img) {
			return mean(img, getMask(img));
		}
	};

	template<class T>
	class Contour_ : Detectable {

	private:
		Mat data;
		Size_<T> _size = Size_<T>();
		Point_<T> _topLeft = Point_<T>();

		int type();
		void calculateDimensions()
		{
		  if (this->_size.width != 0) return;

		  //Calculate size and topLeft at the same time
		  int minX = INT_MAX;
		  int maxX = INT_MIN;
		  int minY = INT_MAX;
		  int maxY = INT_MIN;

		  vector<Point_<T>> points = getPoints();
		  for (Point_<T> p : points) {
		    if (p.x < minX) {
		      minX = p.x;
		    }
		    if (p.y < minY) {
		      minY = p.y;
		    }
		    if (p.x > maxX) {
		      maxX = p.x;
		    }
		    if (p.y > maxY) {
		      maxY = p.y;
		    }
		  }

		  this->_size = Size(maxX - minX, maxY - minY);
		  this->_topLeft = Point(minX, minY);
		}

	public:
		inline Contour_(Mat& data) {
			this->data = data;
		}
		inline Mat& getMat() {
			return data;
		}

		Contour_(vector<Point_<T>>& points)
		{
		  data = Mat(points.size(), 2, type());
		  for (int i = 0; i < points.size(); i++) {
		    data.at<T>(i, 0) = points[i].x;
		    data.at<T>(i, 1) = points[i].y;
		  }
		}

		int points()
		{
		  return data.rows;
		}

		double area()
		{
		  return abs(contourArea(data));
		}

		bool isClosed()
		{
		  return isContourConvex(data);
		}

		Point2d centroid()
		{
		  //C_{\mathrm x} = \frac{1}{6A}\sum_{i=0}^{n-1}(x_i+x_{i+1})(x_i\ y_{i+1} - x_{i+1}\ y_i)
		  //C_{\mathrm y} = \frac{1}{6A}\sum_{i=0}^{n-1}(y_i+y_{i+1})(x_i\ y_{i+1} - x_{i+1}\ y_i)

		  if (points() < 2)
		    return center();

		  double xSum = 0.0;
		  double ySum = 0.0;
		  double area = 0.0;
		  vector<Point_<T>> points = getPoints();

		  for (unsigned int i = 0; i < points.size() - 1; i++) {
		    //cross product, (signed) double area of triangle of vertices (origin,p0,p1)
		    double signedArea = (points[i].x * points[i + 1].y) - (points[i + 1].x * points[i].y);
		    xSum += (points[i].x + points[i + 1].x) * signedArea;
		    ySum += (points[i].y + points[i + 1].y) * signedArea;
		    area += signedArea;
		  }

		  if (area == 0)
		    return center();

		  double coefficient = 3 * area;
		  return Point2d(xSum / coefficient, ySum / coefficient);
		}

		Point2d center()
		{
		  calculateDimensions();
		  return Point2d(_topLeft.x + (_size.width / 2), _topLeft.y + (_size.height / 2));
		}

		double height()
		{
		  calculateDimensions();
		  return _size.height;
		}

		double width()
		{
		  calculateDimensions();
		  return _size.width;
		}

		double top()
		{
		  calculateDimensions();
		  return _topLeft.y;
		}

		double bottom()
		{
		  calculateDimensions();
		  return _topLeft.y + _size.height;
		}

		double left()
		{
		  calculateDimensions();
		  return _topLeft.x;
		}

		double right()
		{
		  calculateDimensions();
		  return _topLeft.x + _size.width;
		}

		Rect_<T> getBoundingRect()
		{
		  return Rect_<T>(top(), left(), width(), height());
		}

		Point_<T> topLeft()
		{
		  calculateDimensions();
		  return _topLeft;
		}

		Size_<T> size()
		{
		  calculateDimensions();
		  return _size;
		}

		double arcLength(bool closed)
		{
		  return cv::arcLength(data, closed);
		}

		vector<Point_<T>> getPoints()
		{
		  vector<Point_<T>> array;
		  if (data.isContinuous()) {
		    array.assign((Point_<T>*)data.datastart, (Point_<T>*)data.dataend);
		  }
		  else {
		    for (int i = 0; i < data.rows; ++i) {
		      array.insert(array.end(), data.ptr<Point_<T>>(i), data.ptr<Point_<T>>(i) + data.cols);
		    }
		  }
		  return array;
		}

		Mat getMask(Mat& img) {
		  //returns binary mask (0 or 1)
		  assert(img.channels() == 1);
		  Mat mask = Mat::zeros(Size(img.cols, img.rows), CV_8U);
		  drawContours(mask, data, -1, (1), 1);
		  return mask;
		}
	};

	typedef Contour_<int> Contour;
	typedef Contour_<int> Contour2i;
	typedef Contour_<float> Contour2f;
	typedef Contour_<double> Contour2d;
}
