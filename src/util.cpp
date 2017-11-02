#include "cvlib/util.h"

#ifndef WINDOWS
#include <X11/Xlib.h>
#endif // WINDOWS

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
        Display* d = XOpenDisplay(NULL);
        Screen* s = DefaultScreenOfDisplay(d);
        cv::Size z = cv::Size(s->width, s->height);
        delete d;
        delete s;
        return z;
#endif
	}
}
