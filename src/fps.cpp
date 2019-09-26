#include "robosub/fps.h"

namespace robosub {
    FPS::FPS() {
        for (int i = 0; i < BUFFER_LENGTH; i++)
            _freqBuffer[i] = 0;
    }

    double FPS::frame() {
        long long time = Time::millis();
        if (_prevTime == 0) _prevTime = time;

        _currentSum -= _freqBuffer[_bufferPos];
        _currentSum += time - _prevTime;
        _freqBuffer[_bufferPos] = time - _prevTime;
        _bufferPos = (_bufferPos + 1) % BUFFER_LENGTH;
        _prevTime = time;
        return fps();
    }

    double FPS::fps() {
        if (_currentSum == 0) return 0;
        return (1.0 / ((double) _currentSum / (double) BUFFER_LENGTH / 1000.0));
    }
}
