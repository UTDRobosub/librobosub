
#pragma once

struct ReadoutData{
	bool valid;
	
	float rtt;
	float cpu;
	float ram;
	
	float accel_x;
	float accel_y;
	float accel_z;
};

void readout(ReadoutData* data);
