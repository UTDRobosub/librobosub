
#pragma once

#include <stdio.h>
#include <stdlib.h> //rand
#include <librobosub/telemetry.h>
#include <librobosub/serial.h>
#include <librobosub/util.h>

using namespace robosub;

void initRobotState();
void updateRobotTelemetry(DataBucket& state);
void updateRobotControls(DataBucket& state);
