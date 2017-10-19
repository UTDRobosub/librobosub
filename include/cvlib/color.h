#pragma once

/*#pragma once

#include "common.h"
#include <opencv2/opencv.hpp>

namespace cvlib
{
	enum ColorSpace
	{
		RGBA,
		RGB,
		HSV,
		GRAY
	};

	class Color abstract
	{
	private:
		Scalar scalar;
		ColorSpace space;

		Color* create(ColorSpace space, Scalar s);

	protected:
		static const int MAX_CONVERSION_LENGTH = 4;
		typedef int Conversion[MAX_CONVERSION_LENGTH * 2 + 1];

		virtual ColorSpace colorspace() = 0;
		virtual int channels() = 0;

		virtual int* conversionsTo(int other) = 0;

	public:
		EXPORT Color(ColorSpace cs, Scalar s);

		EXPORT Color(ColorSpace cs, double a);
		EXPORT Color(ColorSpace cs, double a, double b);
		EXPORT Color(ColorSpace cs, double a, double b, double c);
		EXPORT Color(ColorSpace cs, double a, double b, double c, double d);

		EXPORT Scalar color();

		EXPORT Scalar colorIn(ColorSpace space);

		EXPORT Color* convertTo(ColorSpace to);

		EXPORT bool canConvertTo(ColorSpace to);

		EXPORT static void convertMat(Mat& image, ColorSpace from, ColorSpace to);
	};

	class ColorRGBA : Color
	{
	public:
		EXPORT ColorRGBA(double r, double b, double g, double a);
		EXPORT ColorRGBA(String hexCode);
		EXPORT ColorRGBA(Scalar s);

		inline ColorSpace colorspace() { return ColorSpace::RGBA; }
		inline int channels() { return 4; }

		inline int* conversionsTo(int other) {
			switch (other)
			{
			case ColorSpace::RGB:
				return new Conversion{ COLOR_RGBA2RGB, 3, -1 };
			case ColorSpace::HSV:
				return new Conversion{ COLOR_RGBA2RGB, 3, COLOR_RGB2HSV_FULL, 3, -1 };
			case ColorSpace::GRAY:
				return new Conversion{ COLOR_RGB2GRAY, 1, -1 };
			default:
				return new Conversion{ -1 };
			}
		}
	};

	class ColorRGB : Color
	{
	public:
		static inline ColorSpace colorspace() { return ColorSpace::RGB; }
		static inline int channels() { return 3; }

		static inline int* conversionsTo(int other) {
			switch (other)
			{
			case ColorSpace::RGBA:
				return new Conversion{ COLOR_RGB2RGBA, 4, -1 };
			case ColorSpace::HSV:
				return new Conversion{ COLOR_RGB2HSV_FULL, 3, -1 };
			case ColorSpace::GRAY:
				return new Conversion{ COLOR_RGB2GRAY, 1, -1 };
			default:
				return new Conversion{ -1 };
			}
		}
	};

	class ColorHSV : Color
	{
	public:
		static inline ColorSpace colorspace() { return ColorSpace::HSV; }
		static inline int channels() { return 3; }

		static inline int* conversionsTo(int other) {
			switch (other)
			{
			case ColorSpace::RGB:
				return new Conversion{ COLOR_HSV2RGB_FULL, 3, -1 };
			case ColorSpace::RGBA:
				return new Conversion{ COLOR_HSV2RGB_FULL, 3, COLOR_RGB2RGBA, 4, -1 };
			case ColorSpace::GRAY:
				return new Conversion{ COLOR_HSV2RGB_FULL, 3, COLOR_RGB2GRAY, 1, -1 };
			default:
				return new Conversion{ -1 };
			}
		}
	};

	class ColorGRAY : Color
	{
	public:
		static inline ColorSpace colorspace() { return ColorSpace::GRAY; }
		static inline int channels() { return 1; }

		static inline int* conversionsTo(int other) {
			switch (other)
			{
			case ColorSpace::RGBA:
				return new Conversion{ COLOR_GRAY2RGBA, 4, -1 };
			case ColorSpace::RGB:
				return new Conversion{ COLOR_GRAY2RGB, 3, -1 };
			case ColorSpace::GRAY:
				return new Conversion{ COLOR_GRAY2RGB, 3, COLOR_RGB2HSV_FULL, 3, -1 };
			default:
				return new Conversion{ -1 };
			}
		}
	};
}*/