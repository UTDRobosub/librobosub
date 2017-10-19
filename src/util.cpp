#include "cvlib/util.h"

namespace cvlib {
	void Util::pause()
	{
		std::cin.ignore();
	}
	cv::Size Util::getDesktopResolution()
	{
#ifdef WINDOWS
		return cv::Size(GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
#else
		//TODO NOT IMPLEMENTED YET
#endif
	}
}
