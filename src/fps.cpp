#include "cvlib\fps.h"

double FPS::frame()
{
	if (Time::millis() - _fpsstart>1000)
	{
		_fpsstart = Time::millis();
		_avgfps = 0.7*_avgfps + 0.3*_fps1sec;
		_fps1sec = 0;
	}
	_fps1sec++;
	return _avgfps;
}

double FPS::fps() {
	return _avgfps;
}