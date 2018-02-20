
#include "robosub/networkvideo.h"

namespace robosub{
	const int networkVideo_pixelSize = 2;
	const int networkVideo_packetSize = 512;
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
	
	int pixellocToIndex(int rows, int cols, int loc){
		return (rows*cols) - loc - 1;
		//return loc;
		//return (rows-(loc%cols)) + (loc-(loc%cols));
	}
	
    void SendFrame(UDPS *udps, Mat *frame){
    	int frameid=(lastframeid+1)%0x10000; //2 bytes long
    	lastframeid=frameid;
    	
        int rows = frame->rows;
        int cols = frame->cols;
		
		int len = rows*cols;
        int datalen = len*3; //3 because 3 bytes per pixel, BGR format.
        
        int numpackets = (int)ceil(((float)(len*networkVideo_pixelSize))/((float)networkVideo_packetDataSize));
		
        char *data = (char*)frame->data;
        
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
			
			udps->send(networkVideo_packetSize, packetdata);
        }
    }
    
    Mat *networkVideo_recvFrame=0;
    char networkVideo_recvFrameData[1280*720*3];
	
    Mat *RecvFrame(UDPR *udpr){
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
			int rows =        twobytes(packetdata+2);
			int cols =        twobytes(packetdata+4);
			int packetindex = twobytes(packetdata+6);
			
			//cout<<"res = "<<cols<<"x"<<rows<<endl;
			
			int len = rows*cols;
			int datalen = len*3;
			
			int pixelloc = packetindex*networkVideo_packetDataPixels;
			
			if(networkVideo_recvFrame==0 || rows!=networkVideo_recvFrame->rows || cols!=networkVideo_recvFrame->cols){
				networkVideo_recvFrame = new Mat(rows,cols,CV_8UC3,networkVideo_recvFrameData);
			}
			
			for(int i=0; i<networkVideo_packetDataPixels; i++){
				int dataloc = pixellocToIndex(rows, cols, pixelloc + i)*3;
				
				if(dataloc<datalen && dataloc>=0){
					int pixel = twobytes(packetdata + networkVideo_packetHeadSize + i*networkVideo_pixelSize);
					
					networkVideo_recvFrameData[dataloc + 0] = (pixel>>8)&0b11111000;
					networkVideo_recvFrameData[dataloc + 1] = (pixel>>3)&0b11111000;
					networkVideo_recvFrameData[dataloc + 2] = (pixel<<2)&0b11111000;
				}
			}
			
			//memcpy(networkVideo_recvFrameData+packetloc, packetdata+networkVideo_packetHeadSize, networkVideo_packetDataSize);
    	}
    	
    	return networkVideo_recvFrame;
    }
}
