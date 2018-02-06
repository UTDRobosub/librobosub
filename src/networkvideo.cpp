
#include "robosub/networkvideo.h"

namespace robosub{
	const int packetHeadSize = 8;
	const int packetDataSize = 64;
	const int packetSize = packetHeadSize + packetDataSize;
	
	int lastframeid=0;
	
	int firstbyte(int x){
		return x & 0xFF;
	}
	int secondbyte(int x){
		return (x & 0xFF00) >> 8;
	}
	
    void SendFrame(UDPS *udps, Mat *frame){
    	int frameid=(lastframeid+1)%0x10000; //2 bytes long
    	lastframeid=frameid;
    	
        int rows = frame->rows;
        int cols = frame->cols;
		
		int len = rows*cols;
        int datalen = len*3; //3 because 3 bytes per pixel, BGR format.
        int sendlen = len*2;
		
        char *data = (char*)frame->data;
        char senddata[sendlen];
		
        for(int i=0; i<len; i++){
			int sd = ((data[i*3]&0b11111000)<<11) | ((data[i*3+1]&0b11111000)<<6) | ((data[i*3+2]&0b11111000)<<1) | 0b1;
			//condense the pixels into 2 bytes, 5 bits per color, then a 1 in the LSB position
			//bit format: BBBBBGGG GGRRRRR1
			senddata[i*2]   = secondbyte(sd);
			senddata[i*2+1] = firstbyte (sd);
        }
        
        int numpackets = (int)ceil((float)sendlen/(float)packetDataSize);
	
        for(int i=0; i<numpackets; i++){
			
			char packetdata[packetSize];
			
			int packetindex = i*packetDataSize;
			
			//start the packet data buffer with a header, consisting of: frame id, byte index of this data, rows in picture, cols in picture
			packetdata[0] = secondbyte(frameid);
			packetdata[1] = firstbyte (frameid);
			packetdata[2] = secondbyte(packetindex);
			packetdata[3] = firstbyte (packetindex);
			packetdata[4] = secondbyte(rows);
			packetdata[5] = firstbyte (rows);
			packetdata[6] = secondbyte(cols);
			packetdata[7] = firstbyte (cols);
			
			//copy some of the bytes into the packet data buffer
			for(int j=0; j<packetDataSize; j++){
				packetdata[j+packetHeadSize] = senddata[packetindex+j];
			}
			
			udps->send(packetSize, packetdata);
        }
    }
	
    Mat *RecvFrame(UDPR *udpr){
    	while(true){
			char packetdata[packetSize];
			
			int recvlen;
			udpr->recv(packetSize, recvlen, packetdata);
			
			if(recvlen==0){
				break;
			}else if(recvlen != packetSize){
				cout<<"invalid packet: wrong len "<<recvlen<<endl;
			}
			cout<<"valid packet "<<(int)packetdata[1]<<endl;
			
    	}
    	return 0;
    }
}
