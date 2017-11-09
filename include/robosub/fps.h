#pragma once

#include "common.h"
#include "time.h"

namespace robosub 
{
	class FPS 
	{
	private:
		static const int BUFFER_LENGTH = 7;

		double _freqBuffer[BUFFER_LENGTH]; //list of time deltas
		long long _currentSum = 0; //sum of time deltas
		long long _prevTime = 0;
		int _bufferPos = 0;

	public:

		EXPORT FPS();

		///Calculate the FPS of a single frame
		///Returns current fps
		EXPORT double frame();
		///Get current fps
		EXPORT double fps();
	};
}