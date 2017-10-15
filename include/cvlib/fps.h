#pragma once

#include "common.h"
#include "time.h"

namespace cvlib 
{
	class FPS 
	{
	private:
		long long _fpsstart = 0;
		double _avgfps = 0;
		double _fps1sec = 0;

	public:
		///Calculate the FPS of a single frame
		///Returns current fps
		EXPORT double frame();
		///Get current fps
		EXPORT double fps();
	};
}