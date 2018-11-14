
#include "robosub/serial.h"

namespace robosub {
	
	////////////////////////////////////////////////////////////////////////////////////////////
	//Encoding and decoding; not part of Serial class.
	//Copy and paste this code into an arduino and it will still work.
	
	static const unsigned char SerialCRC8Table[256] ={
		0x00, 0x25, 0x4A, 0x6F, 0x94, 0xB1, 0xDE, 0xFB,
		0x0D, 0x28, 0x47, 0x62, 0x99, 0xBC, 0xD3, 0xF6,
		0x1A, 0x3F, 0x50, 0x75, 0x8E, 0xAB, 0xC4, 0xE1,
		0x17, 0x32, 0x5D, 0x78, 0x83, 0xA6, 0xC9, 0xEC,
		0x34, 0x11, 0x7E, 0x5B, 0xA0, 0x85, 0xEA, 0xCF,
		0x39, 0x1C, 0x73, 0x56, 0xAD, 0x88, 0xE7, 0xC2,
		0x2E, 0x0B, 0x64, 0x41, 0xBA, 0x9F, 0xF0, 0xD5,
		0x23, 0x06, 0x69, 0x4C, 0xB7, 0x92, 0xFD, 0xD8,
		0x68, 0x4D, 0x22, 0x07, 0xFC, 0xD9, 0xB6, 0x93,
		0x65, 0x40, 0x2F, 0x0A, 0xF1, 0xD4, 0xBB, 0x9E,
		0x72, 0x57, 0x38, 0x1D, 0xE6, 0xC3, 0xAC, 0x89,
		0x7F, 0x5A, 0x35, 0x10, 0xEB, 0xCE, 0xA1, 0x84,
		0x5C, 0x79, 0x16, 0x33, 0xC8, 0xED, 0x82, 0xA7,
		0x51, 0x74, 0x1B, 0x3E, 0xC5, 0xE0, 0x8F, 0xAA,
		0x46, 0x63, 0x0C, 0x29, 0xD2, 0xF7, 0x98, 0xBD,
		0x4B, 0x6E, 0x01, 0x24, 0xDF, 0xFA, 0x95, 0xB0,
		0xD0, 0xF5, 0x9A, 0xBF, 0x44, 0x61, 0x0E, 0x2B,
		0xDD, 0xF8, 0x97, 0xB2, 0x49, 0x6C, 0x03, 0x26,
		0xCA, 0xEF, 0x80, 0xA5, 0x5E, 0x7B, 0x14, 0x31,
		0xC7, 0xE2, 0x8D, 0xA8, 0x53, 0x76, 0x19, 0x3C,
		0xE4, 0xC1, 0xAE, 0x8B, 0x70, 0x55, 0x3A, 0x1F,
		0xE9, 0xCC, 0xA3, 0x86, 0x7D, 0x58, 0x37, 0x12,
		0xFE, 0xDB, 0xB4, 0x91, 0x6A, 0x4F, 0x20, 0x05,
		0xF3, 0xD6, 0xB9, 0x9C, 0x67, 0x42, 0x2D, 0x08,
		0xB8, 0x9D, 0xF2, 0xD7, 0x2C, 0x09, 0x66, 0x43,
		0xB5, 0x90, 0xFF, 0xDA, 0x21, 0x04, 0x6B, 0x4E,
		0xA2, 0x87, 0xE8, 0xCD, 0x36, 0x13, 0x7C, 0x59,
		0xAF, 0x8A, 0xE5, 0xC0, 0x3B, 0x1E, 0x71, 0x54,
		0x8C, 0xA9, 0xC6, 0xE3, 0x18, 0x3D, 0x52, 0x77,
		0x81, 0xA4, 0xCB, 0xEE, 0x15, 0x30, 0x5F, 0x7A,
		0x96, 0xB3, 0xDC, 0xF9, 0x02, 0x27, 0x48, 0x6D,
		0x9B, 0xBE, 0xD1, 0xF4, 0x0F, 0x2A, 0x45, 0x60
	};
	
	const int SerialMaxMessageLength = 256;
	
	const char SerialState_Outside        = 1;
	const char SerialState_InMessageFlags = 2;
	const char SerialState_InMessageSN    = 3;
	const char SerialState_InMessageData  = 4;
	const char SerialState_InMessageCRC   = 5;
	
	const char SerialFlags_IncludesAck    = 0b00000001;
	const char SerialFlags_NeedsAck       = 0b00000010;
	const char SerialFlags_NoCRCCheck     = 0b00000100;
	
	struct SerialReceiverState{
		void (*receiveMessageCallback)(char *message, int length, char sequenceNumber, bool needsack);
		void (*receiveAckCallback)(char sequenceNumber);
		
		char state;
		
		bool inCharacterEscape;
		char currentMessageFlags;
		char currentMessageSN;
		char currentMessageCRC;
		char currentMessageComputedCRC;
		
		int currentMessageDataLength;
		char* currentMessageData;
	};
	
	void SerialReceiveChar(SerialReceiverState* state, char rawdata){
		char escdata = rawdata;
		
		if(rawdata=='\\'){
			state->inCharacterEscape = true;
		}else{
			if(state->inCharacterEscape){
				if     (rawdata=='(') escdata = '[';
				else if(rawdata==')') escdata = ']';
				else if(rawdata=='/') escdata = '\\';
				state->inCharacterEscape = false;
			}
			
			if(rawdata=='['){
				state->state = SerialState_InMessageSN;
				state->currentMessageFlags = 0;
				state->currentMessageSN = 0;
				state->currentMessageComputedCRC = 0;
				
				state->currentMessageDataLength = 0;
				
			}else if(state->state==SerialState_InMessageFlags){
				state->currentMessageFlags = escdata;
				
				state->currentMessageComputedCRC = SerialCRC8Table[state->currentMessageComputedCRC ^ escdata];
				
				state->state = SerialState_InMessageSN;
				
			}else if(state->state==SerialState_InMessageSN){
				state->currentMessageSN = escdata;
				
				state->currentMessageComputedCRC = SerialCRC8Table[state->currentMessageComputedCRC ^ escdata];
				
				state->state = SerialState_InMessageData;
				
			}else if(state->state==SerialState_InMessageData){
				if(rawdata!=']'){
					if(state->currentMessageDataLength<SerialMaxMessageLength){
						state->currentMessageData[state->currentMessageDataLength] = escdata;
						state->currentMessageDataLength++;
						
						state->currentMessageComputedCRC = SerialCRC8Table[state->currentMessageComputedCRC ^ escdata];
					}else{
						state->state = SerialState_Outside;
					}
				}else{
					state->state = SerialState_InMessageCRC;
				}
				
			}else if(state->state==SerialState_InMessageCRC){
				state->currentMessageCRC = escdata;
				
				if(state->currentMessageCRC==state->currentMessageComputedCRC || state->currentMessageFlags&SerialFlags_NoCRCCheck){
					if(state->currentMessageFlags&SerialFlags_IncludesAck){
						state->receiveAckCallback(state->currentMessageSN);
					}
					
					bool needsack = false;
					if(state->currentMessageFlags&SerialFlags_NeedsAck){
						needsack = true;
					}
					
					state->receiveMessageCallback(state->currentMessageData, state->currentMessageDataLength, state->currentMessageSN, needsack);
				}
				
				state->state = SerialState_Outside;
				
			}
		}
	}
	
	void SerialSendMessage(char* buffer, int length, int sequenceNumber, bool includesack, bool needsack, void (*sendchar)(char)){
		char flags = 0b00100000;
		if(includesack){ flags = flags|SerialFlags_IncludesAck; }
		if(needsack   ){ flags = flags|SerialFlags_NeedsAck   ; }
		
		char crc;
		
		sendchar('[');
		
		sendchar(flags);
		crc = SerialCRC8Table[crc ^ flags];
		sendchar(sequenceNumber);
		crc = SerialCRC8Table[crc ^ sequenceNumber];
		
		for(int i=0; i<length; i++){
			char data = buffer[i];
			
			if     (data=='[' ){ sendchar('\\'); sendchar('('); }
			else if(data==']' ){ sendchar('\\'); sendchar(')'); }
			else if(data=='\\'){ sendchar('\\'); sendchar('/'); }
			else               { sendchar(data);                }
			
			crc = SerialCRC8Table[crc ^ data];
		}
		
		sendchar(']');
		
		sendchar(crc);
	}
	
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	Serial::Serial(string fn, int baud){
		connected = false;
		
		filename = fn;
		readbuflen = 0;
		readbuf = (char*)malloc(maxreadbuflen);
		
		int ibaud = 0;
		switch(baud){
			case       0: ibaud =       B0; break;
			case      50: ibaud =      B50; break;
			case      75: ibaud =      B75; break;
			case     110: ibaud =     B110; break;
			case     134: ibaud =     B134; break;
			case     150: ibaud =     B150; break;
			case     200: ibaud =     B200; break;
			case     300: ibaud =     B300; break;
			case     600: ibaud =     B600; break;
			case    1200: ibaud =    B1200; break;
			case    1800: ibaud =    B1800; break;
			case    2400: ibaud =    B2400; break;
			case    4800: ibaud =    B4800; break;
			case    9600: ibaud =    B9600; break;
			case   19200: ibaud =   B19200; break;
			case   38400: ibaud =   B38400; break;
			case   57600: ibaud =   B57600; break;
			case  115200: ibaud =  B115200; break;
			case  230400: ibaud =  B230400; break;
			case  460800: ibaud =  B460800; break;
			case  500000: ibaud =  B500000; break;
			case  576000: ibaud =  B576000; break;
			case  921600: ibaud =  B921600; break;
			case 1000000: ibaud = B1000000; break;
			case 1152000: ibaud = B1152000; break;
			case 1500000: ibaud = B1500000; break;
			case 2000000: ibaud = B2000000; break;
			case 2500000: ibaud = B2500000; break;
			case 3000000: ibaud = B3000000; break;
			case 3500000: ibaud = B3500000; break;
			case 4000000: ibaud = B4000000; break;
			default: cout<<"Serial port \'"<<filename<<"\' invalid baud rate specified"<<endl;
		}
		
		file = open(filename.c_str(), O_RDWR | O_NOCTTY);
		if(file<0){
			cout<<"Serial port \'"<<filename<<"\' failed to open: error "<<errno<<endl;
		}else{
			
			memset(&tty, 0, sizeof(tty));
			if(tcgetattr(file, &tty)!=0){
				cout<<"Serial port \'"<<filename<<"\' tty tcgetattr error "<<errno<<endl;
			}else{
				
				cfsetospeed(&tty, ibaud);
				cfsetispeed(&tty, ibaud);
				
				tty.c_cflag &= ~PARENB; //8n1
				tty.c_cflag &= ~CSTOPB;
				tty.c_cflag &= ~CSIZE;
				tty.c_cflag |= CS8;
				tty.c_cflag &= ~CRTSCTS; //no flow control
				
				tty.c_lflag = 0; //no protocol
				tty.c_oflag = 0; //no remap, no delay
				
				tty.c_cc[VMIN] = 0; //non-blocking read
				tty.c_cc[VTIME] = 0; //100 ms read timeout
				
				tty.c_cflag |= CREAD | CLOCAL; //turn on read and ignore control lines
				tty.c_iflag &= ~(IXON | IXOFF | IXANY); //no flow control;
				tty.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG); //raw
				tty.c_oflag &= ~OPOST; //raw
				
				tcflush(file, TCIFLUSH);
				
				if(tcsetattr(file, TCSANOW, &tty)!=0){
					cout<<"Serial port \'"<<filename<<"\' tty tcsetattr error "<<errno<<endl;
				}else{
					connected = true;
				}
			}
		}
		
		receiverstate = new SerialReceiverState();
		receiverstate->currentMessageData = new char[SerialMaxMessageLength];
		
		sendbuf = new char[SerialMaxMessageLength*2+4];
		sendbuflen = 0;
	}
	
	Serial::~Serial(){
		free(readbuf);
		close(file);
		delete(receiverstate->currentMessageData);
	}
	
	//reads all available data into the local buffer
	void Serial::readEntireBuffer(){
		if(!connected){
			// cout<<"Serial port \'"<<filename<<"\' tried to read but not connected"<<endl;
			return;
		}
		
		readbuflen += read(file, readbuf+readbuflen, maxreadbuflen-readbuflen);
		
		if(readbuflen==maxreadbuflen){
			cout<<"Serial port \'"<<filename<<"\' read buffer full"<<endl;
		}
	}
	
	//transmits len chars out of buf
	void Serial::writeLen(char *buf, int len){
		if(!connected){
			cout<<"Serial port \'"<<filename<<"\' tried to write but not connected"<<endl;
			return;
		}
		
		write(file, buf, len);
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//public
	
	//whether it was successfully initialized and is currently operational
	bool Serial::isConnected(){
		return connected;
	}
	
	//empty the buffer of unprocessed received data
	void Serial::flushBuffer(){
		readbuflen = 0;
	}
	
	//process all received data and call the callbacks where needed
	int Serial::receiveAllMessages(){
		readEntireBuffer();
		
		for(int dataidx=0; dataidx<readbuflen; dataidx++){
			SerialReceiveChar(receiverstate, readbuf[dataidx]);
		}
	}
	
	//void SerialSendMessage(char* buffer, int length, int sequenceNumber, bool includesack, bool needsack, void (*sendchar)(char)){
	
	void Serial::appendChar(char c){
		
	}
	
	void Serial::transmitMessageFast(char* message, int length){
		SerialSendMessage(message, length, currentSequenceNumber, false, false, 
	}
	void Serial::transmitMessageReliable(char* message, int length){
		SerialSendMessage
	}
}
