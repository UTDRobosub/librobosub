
#pragma once

#include "common.h"
#include "networkudp.h"

#include <opencv2/opencv.hpp>

namespace robosub{
    void SendFrame(UDPS*, Mat*);
    Mat *RecvFrame(UDPR*, int*);
    Mat *RecvFrame(UDPR*);
}
