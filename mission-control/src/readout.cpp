#include "../include/cvplot/cvplot.h"
#include "main.h"
#include <opencv2/opencv.hpp>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wmissing-noreturn"

struct ContinuousPlotData{
	float* dataPoints;
	int queuePos;
	int queueSize;
	
	ContinuousPlotData(int size){
		dataPoints = new float[size];
		queuePos = 0;
		queueSize = size;
		
		for(int i=0; i<size; i++){
			dataPoints[i] = 0;
		}
	}
	~ContinuousPlotData(){
		delete(dataPoints);
	}
	
	void addPoint(float val){
		dataPoints[queuePos] = val;
		queuePos = (queuePos+1)%queueSize;
	}
	
	void putIntoSeries(cvplot::Series& series){
		series.clear();
		for(int i=0; i<queueSize; i++){
			series.addValue(dataPoints[(queuePos+i)%queueSize]);
		}
	}
};

struct ContinuousPlotSeries{
	ContinuousPlotData* data;
	const char* seriesName;
};

struct ContinuousPlotGraphic{
	ContinuousPlotSeries* series;
	int numSeries;
	
	float xmin;
	float xmax;
	float ymin;
	float ymax;
	
	const char* figureName;
	const char* title;
	
	ContinuousPlotGraphic(const char* figureName_, const char* title_, int numSeries_, float xpos, float ypos, float xsize, float ysize, float xmin_, float xmax_, float ymin_, float ymax_){
		numSeries = 0;
		figureName = figureName_;
		title = title_;
		
		xmin = xmin_;
		xmax = xmax_;
		ymin = ymin_;
		ymax = ymax_;
		
		cvplot::setWindowTitle(figureName, title);
		cvplot::moveWindow    (figureName, xpos, ypos);
		cvplot::resizeWindow  (figureName, xsize, ysize);
		
		series = new ContinuousPlotSeries[numSeries_];
	}
	
	void addSeries(const char* seriesName_, ContinuousPlotData& data){
		series[numSeries].data = &data;
		series[numSeries].seriesName = seriesName_;
		numSeries++;
	}
	
	void show(){
		for(int i=0; i<numSeries; i++){
			series[i].data->putIntoSeries(cvplot::figure(figureName).series(series[i].seriesName));
		}
		cvplot::figure(figureName).showManualBounds(true, xmin, xmax, ymin, ymax);
		waitKey(1);
	}
};

void readout(ThreadData* data){



	int pointsToGraph = 200;
	int graphHeight = 200*data->readout->scale;
	int graphWidth = 600*data->readout->scale;
	
	int curGraphY = -graphHeight;
	
	//params: identifier title numSeries xpos ypos xsize ysize xmin xmax ymin ymax
	ContinuousPlotGraphic rtt("rtt", "Robot RTT", 1, 0, curGraphY+=graphHeight, graphWidth, graphHeight, 0, pointsToGraph, 0, 10);
	ContinuousPlotData rtt_rtt(pointsToGraph); rtt.addSeries("RTT ms", rtt_rtt);
	
	ContinuousPlotGraphic cpu("cpu", "Robot Resource Usage", 1, 0, curGraphY+=graphHeight, graphWidth, graphHeight, 0, pointsToGraph, 0, 100);
	ContinuousPlotData cpu_cpu(pointsToGraph); cpu.addSeries("CPU %"   , cpu_cpu);
	ContinuousPlotData cpu_ram(pointsToGraph); cpu.addSeries("Memory %", cpu_ram);
	
	ContinuousPlotGraphic accel("accel", "Acceleration", 3, 0, curGraphY+=graphHeight, graphWidth, graphHeight, 0, pointsToGraph, -2, 2);
	ContinuousPlotData accel_x(pointsToGraph); accel.addSeries("X G", accel_x);
	ContinuousPlotData accel_y(pointsToGraph); accel.addSeries("Y G", accel_y);
	ContinuousPlotData accel_z(pointsToGraph); accel.addSeries("Z G", accel_z);
	
	ContinuousPlotGraphic stuff("stuff", "Stuff", 1, 0, curGraphY+=graphHeight, graphWidth, graphHeight, 0, pointsToGraph, 0, 256);
	ContinuousPlotData stuff_stuff(pointsToGraph); stuff.addSeries("Stuff", stuff_stuff);
	
	unsigned char i;
	
	while(true){
		if(data->readout->valid){
			rtt_rtt.addPoint(data->readout->rtt);
			
			cpu_cpu.addPoint(data->readout->cpu);
			cpu_ram.addPoint(data->readout->ram);
			
			accel_x.addPoint(data->readout->accel_x);
			accel_y.addPoint(data->readout->accel_y);
			accel_z.addPoint(data->readout->accel_z);
		}
		
		stuff_stuff.addPoint(i+=5);

		rtt.show();
		cpu.show();
		accel.show();
		stuff.show();

		robosub::Time::waitMillis(10);
	}
}

#pragma clang diagnostic pop