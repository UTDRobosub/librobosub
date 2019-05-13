#pragma once

#include <robosub/robosub.h>
#include <robosub/networktcp.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <mutex>

extern bool running;
extern bool refresh;
extern mutex drawLock;
extern VideoWriter videoFiles[];

extern const int NUMFEEDS;
extern const int PORT[5];
extern const String VIDEO_ADDR;
extern String FILE_PREFIX;