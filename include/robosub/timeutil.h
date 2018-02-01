#pragma once

#include "common.h"
#include <thread>

namespace robosub
{
	class Time
	{
	private:
		static void _initCounter();

	public:
		///Pause the running thread for n microseconds
		EXPORT static void waitMicros(long long micros);
		///Pause the running thread for n milliseconds
		EXPORT static void waitMillis(long long millis);
		///Pause the running thread for n seconds
		EXPORT static void waitSeconds(long seconds);
		///Pause the running thread for n seconds
		EXPORT static void waitSeconds(double seconds);
		///Pause the running thread just until other threads have finished
		EXPORT static void wait();
		///Get accurate timestamp in milliseconds
		EXPORT static long long millis();
	};

	class Stopwatch
	{
	private:
		long long resetTime;

	public:
		///Create and start stopwatch
		EXPORT Stopwatch();
		///Reset stopwatch
		EXPORT void reset();
		///Get time elapsed in milliseconds
		EXPORT long long elapsed();
	};
}
