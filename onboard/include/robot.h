#pragma once

#include <stdio.h>
#include <stdlib.h> //rand
#include <robosub/telemetry.h>
#include <robosub/serial.h>
#include <robosub/util.h>

void initRobotState();
void updateRobotTelemetry(DataBucket& state);
void updateRobotControls(DataBucket& state);
