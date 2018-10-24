
#pragma once

#include "common.h"
#include "networkudp.h"

#include <opencv2/opencv.hpp>

namespace robosub{
	const int NetworkVideo_MostRecentFrameCount = 1;
	
	class NetworkVideoFrameReceiver{
		Mat* bufferFrames[NetworkVideo_MostRecentFrameCount];
		
		bool* bufferFramesReceivedPacket[NetworkVideo_MostRecentFrameCount];
		int* bufferFramesMostRecentFrameId[NetworkVideo_MostRecentFrameCount];
		
		int bufferFramesGoodPacketCount[NetworkVideo_MostRecentFrameCount];
		int bufferFramesTotalFrameId[NetworkVideo_MostRecentFrameCount];
		
		int rows;
		int cols;
		int mostRecentFrameId;
		bool initialized;
		int packetsPerFrame;
		UDPR *udpr;
		
		void uninitialize(){
			for(int i=0; i<NetworkVideo_MostRecentFrameCount; i++){
				bufferFrames[i] = 0;
				bufferFramesReceivedPacket[i] = 0;
				bufferFramesMostRecentFrameId[i] = 0;
				bufferFramesGoodPacketCount[i] = 0;
				bufferFramesTotalFrameId[i] = 0;
			}
			rows = 0;
			cols = 0;
			packetsPerFrame = 0;
			mostRecentFrameId = -1;
			initialized = false;
		}
		
		public:
		NetworkVideoFrameReceiver(UDPR& iudpr){
			uninitialize();
			udpr = &iudpr;
		}
		~NetworkVideoFrameReceiver(){
			for(int i=0; i<NetworkVideo_MostRecentFrameCount; i++){
				delete(bufferFrames[i]);
				delete(bufferFramesReceivedPacket[i]);
				delete(bufferFramesMostRecentFrameId[i]);
			}
		}
		bool isInitialized();
		Mat* getBestFrame();
		int updateReceiveFrame();
	};
	
    void SendFrame(UDPS&, Mat&);
}
