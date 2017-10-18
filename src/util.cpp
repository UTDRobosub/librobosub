#include "cvlib/util.h"

namespace cvlib {
	void Util::pause()
	{
		std::cin.ignore();
	}

	template<typename T>
	String Util::toStringWithPrecision(const T value, const int n)
	{
		std::ostringstream out;
		out << std::setprecision(n) << value;
		return out.str();
	}
}
