#include "robosub/util.h"

namespace robosub {

	void Util::pause()
	{
		std::cin.ignore();
	}
	cv::Size Util::getDesktopResolution()
	{
#ifdef WINDOWS
		return cv::Size(GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
#else
        Display* d = XOpenDisplay(NULL);
        Screen* s = DefaultScreenOfDisplay(d);
        cv::Size z = cv::Size(s->width, s->height);
        //delete s;
        //XCloseDisplay(d);
        return z;
#endif
	}
}
