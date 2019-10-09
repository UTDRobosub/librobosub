#pragma once

#include "controller.h"
#include "robosub/robosub.h"
#include "robosub/networktcp.h"
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <mutex>

extern bool running;
extern bool refresh;
extern long controllerTime;
extern mutex drawLock;

struct ReadoutData{
    int scale;

    bool valid;

    float rtt;
    float cpu;
    float ram;

    float accel_x;
    float accel_y;
    float accel_z;
};

struct ThreadData {
    ReadoutData* readout;
    int networkFeeds;
    int port;
    String filePrefix;
    String networkHost;
    Controller *controller1;
    Controller *controller2;
};

void control(ThreadData*);
void network(ThreadData*);
void readout(ThreadData*);