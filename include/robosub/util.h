#pragma once

#include "common.h"
#include <iostream>
#include <string>
#include "opencv2/opencv.hpp"

namespace robosub {
	class Util {
	public:
		///Wait for any keypress
		EXPORT static void pause();

		template <typename T>
		inline static String toStringWithPrecision(const T value, const int n = 4) {
			std::ostringstream out;
			out << std::setprecision(n) << value;
			return out.str();
		}
		
		EXPORT static cv::Size getDesktopResolution();
	};	

	template EXPORT String Util::toStringWithPrecision(const float value, const int n);
	template EXPORT String Util::toStringWithPrecision(const double value, const int n);
	template EXPORT String Util::toStringWithPrecision(const long double value, const int n);
}