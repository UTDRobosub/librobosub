
#include "robosub/networkvideo.h"
#include "robosub/timeutil.h"

namespace robosub{
	const int networkVideo_numFrameIds = 0x10000;
	const int networkVideo_pixelSize = 2;
	const int networkVideo_packetSize = 1000;
	const int networkVideo_packetHeadSize = 8;
	const int networkVideo_packetDataSize = networkVideo_packetSize-networkVideo_packetHeadSize;
	const int networkVideo_packetDataPixels = (int)floor(((float)networkVideo_packetDataSize)/((float)networkVideo_pixelSize));
	
	int lastframeid=0;
	
	int firstbyte(int x){
		return x & 0xFF;
	}
	int secondbyte(int x){
		return (x & 0xFF00) >> 8;
	}
	int thirdbyte(int x){
		return (x & 0xFF0000) >> 16;
	}
	int fourthbyte(int x){
		return (x & 0xFF000000) >> 24;
	}
	int twobytes(char *x){
		return (((unsigned char)*x)<<8) | (((unsigned char)*(x+1)));
	}
	int threebytes(char *x){
		return (((unsigned char)*x)<<16) | (((unsigned char)*(x+1))<<8) | (((unsigned char)*(x+2)));
	}
	int fourbytes(char *x){
		return (((unsigned char)*x)<<24) | (((unsigned char)*(x+1))<<16) | (((unsigned char)*(x+2))<<8) | (((unsigned char)*(x+3)));
	}
	
	int getPixelInFrame(char *data, int rows, int cols, int x, int y){
		return threebytes(data + (y*cols + x)*3);
	}
	int putPixelInFrame(char *data, int rows, int cols, int x, int y, int p){
		data[(y*cols + x)*3 + 0] = firstbyte (p);
		data[(y*cols + x)*3 + 1] = secondbyte(p);
		data[(y*cols + x)*3 + 2] = thirdbyte (p);
	}
	
	const int pixellocMod = 1040807;
	
	inline int pixellocToIndex(int rows, int cols, int loc){
		//return (rows*cols) - loc - 1;
		
		return (int)((long long)loc)*((long long)pixellocMod)%(((long long)rows)*((long long)cols));
	}
	
	//transmits the frame over the NetworkUdp UDPS
    void SendFrame(UDPS& udps, Mat& frame){
    	int frameid=(lastframeid+1)%0x10000; //2 bytes long
    	lastframeid=frameid;
    	
        int rows = frame.rows;
        int cols = frame.cols;
		
		int len = rows*cols;
        int datalen = len*3; //3 because 3 bytes per pixel, BGR format.
        
        int numpackets = (int)ceil(((float)(len*networkVideo_pixelSize))/((float)networkVideo_packetDataSize));
		
        char *data = (char*)frame.data;
        
        for(int i=0; i<numpackets; i++){
			
			char packetdata[networkVideo_packetSize];
			
			int packetindex = i;
			int pixelloc = packetindex*networkVideo_packetDataPixels;
			
			//start the packet data buffer with a header, consisting of: frame id, index of this data, rows in picture, cols in picture
			packetdata[0] = secondbyte(frameid);
			packetdata[1] = firstbyte (frameid);
			packetdata[2] = secondbyte(rows);
			packetdata[3] = firstbyte (rows);
			packetdata[4] = secondbyte(cols);
			packetdata[5] = firstbyte (cols);
			packetdata[6] = secondbyte(packetindex);
			packetdata[7] = firstbyte (packetindex);
			
			//copy some of the bytes into the packet data buffer
			for(int j=0; j<networkVideo_packetDataPixels; j++){
				int dataloc = pixellocToIndex(rows, cols, pixelloc + j)*3;
				int pixel = 0;
				
				//condense the pixels into 2 bytes, 5 bits per color, then a 1 in the LSB position
				//bit format: BBBBBGGG GGRRRRR1
				if(dataloc<datalen && dataloc>=0){
					
					pixel =
						((data[dataloc + 0]&0b11111000)<<8) |
						((data[dataloc + 1]&0b11111000)<<3) |
						((data[dataloc + 2]&0b11111000)>>2) |
						0b1
					;
				}
				
				packetdata[networkVideo_packetHeadSize + j*networkVideo_pixelSize + 0] = secondbyte(pixel);
				packetdata[networkVideo_packetHeadSize + j*networkVideo_pixelSize + 1] = firstbyte (pixel);
			}
			
			udps.send(networkVideo_packetSize, packetdata);
        }
    }
    
    //int lastFrameId = 0;
    //int framePacketCount[256];
    //const int framePacketMin = 100;
    
    bool NetworkVideoFrameReceiver::isInitialized(){
		return initialized;
    }
	
	bool frameIdMoreRecent(int curr, int next){
		return (next-curr)%networkVideo_numFrameIds > (curr-next)%networkVideo_numFrameIds;
	}
	
	bool frameIdMoreRecentOrEqual(int curr, int next){
		return (next-curr)%networkVideo_numFrameIds >= (curr-next)%networkVideo_numFrameIds;
	}
	
	//reads all available frame data from the UDPR and puts it into the frame, returning by reference the number of packets read
	//returns the frame, if you specify 0 for the frame it will be assigned to a newly created Mat whenever the first packet is received
    int NetworkVideoFrameReceiver::updateReceiveFrame(){
		int packetsReceived = 0;
		
    	while(true){
			char packetdata[networkVideo_packetSize];
			
			int recvlen;
			int err;
			if(err=udpr->recv(networkVideo_packetSize, recvlen, packetdata)){
				cout<<"RecvFrame: socket.recv error code "<<err<<endl;
				return 0;
			}
			
			if(recvlen==0){
				break;
			}else if(recvlen != networkVideo_packetSize){
				cout<<"RecvFrame: Invalid packet; Incorrect length = "<<recvlen<<endl;
			}
			
			int frameid =     twobytes(packetdata+0);
			int rrows =       twobytes(packetdata+2);
			int rcols =       twobytes(packetdata+4);
			int packetindex = twobytes(packetdata+6);
			
			int len = rrows*rcols;
			int datalen = len*3;
			
			int pixelloc = packetindex*networkVideo_packetDataPixels;
			
			packetsPerFrame = (int)ceil(((float)(len*networkVideo_pixelSize))/((float)networkVideo_packetDataSize));
		
			int currentFrameIdx = frameid%NetworkVideo_MostRecentFrameCount;
			
			/*
			if(networkVideo_recvFrame==0 || rows!=networkVideo_recvFrame->rows || cols!=networkVideo_recvFrame->cols){
				networkVideo_recvFrameData = (char*)malloc(rows*cols*3);
				networkVideo_recvFrame = new Mat(rows,cols,CV_8UC3,networkVideo_recvFrameData);
			}*/
			
			if(initialized && (rrows!=rows || rcols!=cols)){
				cout<<"RecvFrame: Frame received had wrong rows or cols for the given Mat";
				uninitialize();
			}
			
			if(!initialized){
				for(int i=0; i<NetworkVideo_MostRecentFrameCount; i++){
					char* newframedata = (char*)malloc(rrows*rcols*3);
					bufferFrames[i] = new Mat(rrows, rcols, CV_8UC3, newframedata);
					bufferFramesReceivedPacket[i] = new bool[packetsPerFrame];
					bufferFramesMostRecentFrameId[i] = new int[packetsPerFrame];
				}
				char* newframedata = (char*)malloc(rrows*rcols*3);
				bufferFrameLatest = new Mat(rrows, rcols, CV_8UC3, newframedata);
				rows = rrows;
				cols = rcols;
				initialized = true;
				
				cout<<"Creating new frame of size "<<cols<<"x"<<rows<<endl;
			}
			char* recvFrameData = (char*)bufferFrames[currentFrameIdx]->data;
			char* lastFrameData = (char*)bufferFrameLatest->data;
			
			if(frameIdMoreRecent(mostRecentFrameId, frameid)){
				mostRecentFrameId = frameid;
			}
			
			if(frameIdMoreRecentOrEqual(bufferFramesMostRecentFrameId[currentFrameIdx][packetindex], frameid)){
				bufferFramesMostRecentFrameId[currentFrameIdx][packetindex] = frameid;
				
				if(frameIdMoreRecent(bufferFramesTotalFrameId[currentFrameIdx], frameid)){
					bufferFramesTotalFrameId[currentFrameIdx] = frameid;
					bufferFramesGoodPacketCount[currentFrameIdx] = 0;
				}
				if(frameid==bufferFramesTotalFrameId[currentFrameIdx]){
					bufferFramesGoodPacketCount[currentFrameIdx]++;
				}
			}
			
			for(int i=0; i<networkVideo_packetDataPixels; i++){
				int dataloc = pixellocToIndex(rrows, rcols, pixelloc + i)*3;
				
				if(dataloc<datalen && dataloc>=0){
					int pixel = twobytes(packetdata + networkVideo_packetHeadSize + i*networkVideo_pixelSize);
					
					recvFrameData[dataloc + 0] = (pixel>>8)&0b11111000;
					recvFrameData[dataloc + 1] = (pixel>>3)&0b11111000;
					recvFrameData[dataloc + 2] = (pixel<<2)&0b11111000;
					
					lastFrameData[dataloc + 0] = (pixel>>8)&0b11111000;
					lastFrameData[dataloc + 1] = (pixel>>3)&0b11111000;
					lastFrameData[dataloc + 2] = (pixel<<2)&0b11111000;
				}
			}
			
			packetsReceived++;
    	}
    	
    	return packetsReceived;
    }
    
    Mat* NetworkVideoFrameReceiver::getBestFrame(){
		int maxgood = 0;
		int maxgoodidx = 0;
		for(int i=0; i<NetworkVideo_MostRecentFrameCount; i++){
			if(bufferFramesGoodPacketCount[i]>=maxgood){
				maxgoodidx = i;
				maxgood = bufferFramesGoodPacketCount[i];
			}
		}
		
		return bufferFrames[maxgoodidx];
    }
    
    Mat* NetworkVideoFrameReceiver::getLatestFrame(){
		return bufferFrameLatest;
    }
}
