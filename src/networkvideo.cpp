
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
			int dataloc = packetindex*networkVideo_packetDataPixels*3;
			
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
				int sd = ((data[dataloc + j*3 + 0]&0b11111000)<<11) | ((data[dataloc + j*3 + 1]&0b11111000)<<6) | ((data[dataloc + j*3 + 2]&0b11111000)<<1) | 0b1;
				//condense the pixels into 2 bytes, 5 bits per color, then a 1 in the LSB position
				//bit format: BBBBBGGG GGRRRRR1
				packetdata[networkVideo_packetHeadSize + j*networkVideo_pixelSize + 0] = secondbyte(sd);
				packetdata[networkVideo_packetHeadSize + j*networkVideo_pixelSize + 1] = firstbyte (sd);
			}
			
			udps->send(networkVideo_packetSize, packetdata);
		
        }
    }
    
    Mat *networkVideo_recvFrame=0;
    char networkVideo_recvFrameData[1280*720*4];
	
    Mat *RecvFrame(UDPR *udpr){
    	while(true){
			char packetdata[networkVideo_packetSize];
			
			int recvlen;
			int err;
			if(err=udpr->recv(networkVideo_packetSize, recvlen, packetdata)){
				cout<<"recv err "<<err<<endl;
				return 0;
			}
			
			if(recvlen==0){
				cout<<"no data"<<endl;
				break;
			}else if(recvlen != networkVideo_packetSize){
				cout<<"invalid packet: wrong len "<<recvlen<<endl;
			}
			
			int frameid =     twobytes(packetdata+0);
			int rows =        twobytes(packetdata+2);
			int cols =        twobytes(packetdata+4);
			int packetindex = twobytes(packetdata+6);
			
			int dataloc = packetindex*networkVideo_packetDataPixels*3;
			
			//cout<<"packet " << frameid << " index " << packetloc << " rows=" << rows << " cols=" << cols << "."<<endl;
			
			if(networkVideo_recvFrame==0){
				networkVideo_recvFrame = new Mat(rows,cols,CV_8UC3,networkVideo_recvFrameData);
			}
			
			for(int i=0; i<networkVideo_packetDataPixels; i++){
				int pixel = twobytes(packetdata + networkVideo_packetHeadSize + i*networkVideo_pixelSize);
				networkVideo_recvFrameData[dataloc + i*3 + 0] = (pixel & 0b1111100000000000) >> 8;
				networkVideo_recvFrameData[dataloc + i*3 + 1] = (pixel & 0b0000011111000000) >> 3;
				networkVideo_recvFrameData[dataloc + i*3 + 2] = (pixel & 0b0000000000111110) << 2;
			}
			
			//memcpy(networkVideo_recvFrameData+packetloc, packetdata+networkVideo_packetHeadSize, networkVideo_packetDataSize);
    	}
    	
    	cout<<"return from recvframe"<<endl;
    	
    	return networkVideo_recvFrame;
    }
}
