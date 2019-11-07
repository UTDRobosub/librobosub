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

const int NUMFEEDS = 2;
const int PORT[5] = {8500, 8501, 8502, 8503, 8504};
const String VIDEO_ADDR = "127.0.0.1";
extern String FILE_PREFIX;