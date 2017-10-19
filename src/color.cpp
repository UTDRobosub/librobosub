/*#include "cvlib/color.h"

namespace cvlib 
{
	Color* Color::create(ColorSpace space, Scalar s)
	{
		switch (space)
		{
		case ColorSpace::RGBA:
			return new ColorRGBA(s);
		case ColorSpace::RGB:
			return new ColorRGB(s);
		case ColorSpace::GRAY:
			return new ColorGRAY(s);
		case ColorSpace::HSV:
			return new ColorHSV(s);
		}
	}

	Color::Color(ColorSpace cs, Scalar s)
	{
		assert(scalar.channels == this->channels());

		this->scalar = s;
		this->space = cs;
	}
	Color::Color(ColorSpace cs, double a)
	{
		assert(this->channels() >= 1);

		this->scalar = Scalar(a);
		this->space = cs;
	}
	Color::Color(ColorSpace cs, double a, double b)
	{
		assert(this->channels() >= 2);

		this->scalar = Scalar(a, b, 0.0, 0.0);
		this->space = cs;
	}
	Color::Color(ColorSpace cs, double a, double b, double c)
	{
		assert(this->channels() >= 3);

		this->scalar = Scalar(a, b, c, 0.0);
		this->space = cs;
	}
	Color::Color(ColorSpace cs, double a, double b, double c, double d)
	{
		assert(this->channels() == 4);

		this->scalar = Scalar(a, b, c, d);
		this->space = cs;
	}
	Scalar Color::color()
	{
		return scalar;
	}
	Scalar Color::colorIn(ColorSpace space)
	{
		int* conversions = this->conversionsTo(space);
		Scalar color = Scalar(scalar);
		int channelsIn = this->channels();
		for (int count = 0; ; count++) {
			int id = conversions[2 * count];
			if (id == -1) break;
			int channelsOut = conversions[2 * count + 1];
			cvtColor(color, color, id, channelsOut);
			int channelsIn = channelsOut;
		}
		return color;
	}
	Color* Color::convertTo(ColorSpace to)
	{
		
		return this;
	}
	bool Color::canConvertTo(ColorSpace to)
	{
		int* conversions = this->conversionsTo(to);
		return conversions[0] != -1;
	}
	void Color::convertMat(Mat& image, ColorSpace from, ColorSpace to)
	{

	}
}*/