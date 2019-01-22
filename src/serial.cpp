
#include "robosub/serial.h"
#include "robosub/timeutil.h"

#include "robosub-serial.h"

namespace robosub {
	
	/////////////////////////////////////////////////////////////////////////////
	//Static callbacks used by robosub-serial library
	
	void serialDelayMs(void* instance, int ms){
		Time::waitMillis(ms);
	}
	
	void serialSendChar(void* instance, char data, bool terminate){
		Serial* ser = (Serial*)instance;
		
		ser->appendChar(data);
		
		if(terminate){
			ser->transmitBuffer();
		}
	}
	
	void serialPollReceive(void* instance){
		Serial* ser = (Serial*)instance;
		
		ser->receiveAllMessages();
	}
	
	void serialOnReceiveMessage(void* instance, char* message, int length, bool needsresponse, char** response, int* responselength){
		Serial* ser = (Serial*)instance;
		
		ser->onReceiveMessage(message, length, needsresponse, response, responselength);
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//Serial class private
	
	Serial::Serial(string fn, int baud, void (*_receiveMessageCallback)(char* message, int length, bool needsrepsonse, char** response, int* responsepength), bool useprotocol){
		connected = false;

		useprotocol = useprotocol;
		
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
		
		sendbuf = new char[Serial_MaxMessageLength*2+4];
		sendbuflen = 0;
		
		state = Serial_NewState(this, useprotocol, serialOnReceiveMessage, serialSendChar, serialDelayMs, serialPollReceive);

		receiveMessageCallback = _receiveMessageCallback;
	}
	
	Serial::~Serial(){
		free(readbuf);
		close(file);
		Serial_DeleteState(state);
	}
	
	//reads all available data into the local buffer
	void Serial::readEntireBuffer(){
		if(!connected){
			// cout<<"Serial port \'"<<filename<<"\' tried to read but not connected"<<endl;
			return;
		}
		
		readbuflen += read(file, readbuf+readbuflen, maxreadbuflen-readbuflen);
		
		if(readbuflen>=maxreadbuflen){
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
	//Serial class public for use by library
	
	//whether it was successfully initialized and is currently operational
	bool Serial::isConnected(){
		return connected;
	}
	
	//empty the buffer of unprocessed received data
	void Serial::flushBuffer(){
		readbuflen = 0;
	}
	
	//called when it receives a message from receiveAllMessages
	void Serial::onReceiveMessage(char* message, int length, bool needsresponse, char** response, int* responselength){
		receiveMessageCallback(message, length, needsresponse, response, responselength);
	}
	
	void Serial::appendChar(char data){
		sendbuf[sendbuflen] = data;
		sendbuflen++;
		
		if(sendbuflen>=Serial_MaxMessageLength*2+4){
			cout<<"Serial send buffer overflow"<<endl;
			sendbuflen--;
		}
	}
	
	void Serial::transmitBuffer(){
		writeLen(sendbuf, sendbuflen);
		sendbuflen = 0;
	}
	
	/////////////////////////////////////////////////////////////////////////////
	//Public for use by user
	
	//process all received data and call the callbacks where needed
	int Serial::receiveAllMessages(){
		readEntireBuffer();
		
		for(int dataidx=0; dataidx<readbuflen; dataidx++){
			Serial_ReceiveChar(state, readbuf[dataidx]);
		}

		flushBuffer();
	}
	
	void Serial::transmitMessageFast(char* message, int length){
		Serial_SendMessage(state, message, length, false);
	}
	void Serial::transmitMessageReliable(char* message, int length){
		Serial_SendMessage(state, message, length, true);
	}
}
