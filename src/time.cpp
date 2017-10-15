#include "cvlib/time.h"

namespace cvlib {

#include <sys/timeb.h>
#if defined(WINDOWS)
#include <windows.h>
	static bool _qpcInited = false;
	static long long PCFreq = 0.0;
	static __int64 CounterStart = 0;
	void Time::_initCounter()
	{
		LARGE_INTEGER li;
		if (!QueryPerformanceFrequency(&li))
		{
			throw "High accuracy clock failed to initialize";
		}
		PCFreq = long long(li.QuadPart) / 1000ll;
		_qpcInited = true;
	}
	long long Time::millis()
	{
		if (!_qpcInited) Time::_initCounter();
		LARGE_INTEGER li;
		QueryPerformanceCounter(&li);
		return long long(li.QuadPart) / PCFreq;
	}
#elif defined(UNIX)
	void Time::_initCounter()
	{
		return;
	}
	long long Time::millis()
	{
		struct timespec t;
		clock_gettime(CLOCK_MONOTONIC, &t);
		return (t.tv_sec * 1000) + (t.tv_nsec*1e-6);
	}
#endif

	void Time::waitMicros(long long micros) {
		this_thread::sleep_for(std::chrono::microseconds(micros));
	}

	void Time::waitMillis(long long millis) {
		this_thread::sleep_for(std::chrono::milliseconds(millis));
	}

	void Time::waitSeconds(long seconds) {
		this_thread::sleep_for(std::chrono::seconds(seconds));
	}

	void Time::waitSeconds(double seconds) {
		this_thread::sleep_for(std::chrono::microseconds((long long)(seconds * 1000000.0)));
	}

	void Time::wait() {
		this_thread::yield();
	}

	Stopwatch::Stopwatch()
	{
		reset();
	}

	void Stopwatch::reset()
	{
		resetTime = Time::millis();
	}

	long long Stopwatch::elapsed()
	{
		return Time::millis() - resetTime;
	}
}