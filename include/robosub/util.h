#pragma once

#include "common.h"
#include <iostream>
#include <string>
#include <regex>
#include "opencv2/opencv.hpp"
#include <sys/stat.h>

#ifndef WINDOWS
#include <X11/Xlib.h>
#endif

namespace robosub {
	class Util {
	public:
		///Struct for regex matches
		struct Match {
			string str;
			vector<string> groups;

			Match(string str) {
				this->str = str;
			}
			Match(string str, vector<string> groups) {
				this->str = str;
				this->groups = groups;
			}
		};

		///Wait for any keypress
		EXPORT static void pause();

		///Check if directory exists
		EXPORT static bool directoryExists(string path);

		///Check if file exists
		EXPORT static bool fileExists(string path);

		///Split string by character
		EXPORT static vector<string> splitString(string s, char by);

		///return all matches of regex expression
		///first entry is the string itelf
		///returns
		EXPORT static vector<Match> regex(string pattern, string test);

		///return number to certain decimal precision
		template <typename T>
		inline static String toStringWithPrecision(const T value, const int n = 4) {
			std::ostringstream out;
			out << std::setprecision(n) << value;
			return out.str();
		}

		///Get resolution of screen on currently running machine
		EXPORT static Size getDesktopResolution();
	};

	template EXPORT String Util::toStringWithPrecision(const float value, const int n);
	template EXPORT String Util::toStringWithPrecision(const double value, const int n);
	template EXPORT String Util::toStringWithPrecision(const long double value, const int n);
}
