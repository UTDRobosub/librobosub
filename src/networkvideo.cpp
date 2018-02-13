
#include "robosub/networkvideo.h"

namespace robosub{
	const int networkVideo_packetSize = 512;
	const int networkVideo_packetHeadSize = 8;
	const int networkVideo_packetDataSize = networkVideo_packetSize-networkVideo_packetHeadSize;
	
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
	
    void SendFrame(UDPS *udps, Mat *frame){
    	int frameid=(lastframeid+1)%0x10000; //2 bytes long
    	lastframeid=frameid;
    	
        int rows = frame->rows;
        int cols = frame->cols;
		
		int len = rows*cols;
        int datalen = len*3; //3 because 3 bytes per pixel, BGR format.
        
        int numpackets = (int)ceil((float)(len*3)/(float)networkVideo_packetDataSize);
        int sendlen = numpackets*networkVideo_packetDataSize;
		
        char *data = (char*)frame->data;
        char senddata[sendlen];
		
		/*
        for(int i=0; i<len; i++){
			int sd = ((data[i*3]&0b11111000)<<11) | ((data[i*3+1]&0b11111000)<<6) | ((data[i*3+2]&0b11111000)<<1) | 0b1;
			//condense the pixels into 2 bytes, 5 bits per color, then a 1 in the LSB position
			//bit format: BBBBBGGG GGRRRRR1
			senddata[i*2]   = secondbyte(sd);
			senddata[i*2+1] = firstbyte (sd);
        }*/
        
        memcpy(senddata,data,datalen);
		
        for(int i=0; i<numpackets; i++){
			
			char packetdata[networkVideo_packetSize];
			
			int packetindex = i;
			int packetloc = i*networkVideo_packetDataSize;
			
			//start the packet data buffer with a header, consisting of: frame id, byte index of this data, rows in picture, cols in picture
			packetdata[0] = secondbyte(frameid);
			packetdata[1] = firstbyte (frameid);
			packetdata[2] = secondbyte(rows);
			packetdata[3] = firstbyte (rows);
			packetdata[4] = secondbyte(cols);
			packetdata[5] = firstbyte (cols);
			packetdata[6] = secondbyte(packetindex);
			packetdata[7] = firstbyte (packetindex);
			
			//copy some of the bytes into the packet data buffer
			/*for(int j=0; j<networkVideo_packetDataSize; j++){
				packetdata[j+networkVideo_packetHeadSize] = senddata[packetloc+j];
			}*/
			
			cout<<"send "<<packetloc<<endl;
			
			memcpy(packetdata+networkVideo_packetHeadSize, senddata+packetloc, networkVideo_packetDataSize);
			
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
			
			int packetloc = packetindex*networkVideo_packetDataSize;
			
			cout<<"packet " << frameid << " index " << packetloc << " rows=" << rows << " cols=" << cols << "."<<endl;
			
			if(networkVideo_recvFrame==0){
				networkVideo_recvFrame = new Mat(rows,cols,CV_8UC3,networkVideo_recvFrameData);
			}
			
			memcpy(networkVideo_recvFrameData+packetloc, packetdata+networkVideo_packetHeadSize, networkVideo_packetDataSize);
    	}
    	
    	cout<<"return from recvframe"<<endl;
    	
    	return networkVideo_recvFrame;
    }
}
