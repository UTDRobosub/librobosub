
#pragma once

#include "common.h"

#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

namespace robosub {
	const int Serial_MaxBufferLen = 65536;
	
	struct SerialReceiverState;
	
	class Serial{
		string filename;
		
		int file;
		struct termios tty;
		
		bool connected;
		
		char *readbuf;
		int readbuflen;
		const int maxreadbuflen = Serial_MaxBufferLen;
		
		int currentSequenceNumber;
		char* sendbuf;
		int sendbuflen;
		
		SerialReceiverState* receiverstate;
		
		void writeLen(char*, int);
		void readEntireBuffer();
		
		public:
		Serial(string, int);
		~Serial();
		
		bool isConnected();
		
		void flushBuffer();
		
		int receiveAllMessages();
		
		void transmitMessageFast(char* message, int length);
		void transmitMessageReliable(char* message, int length);
	};
}
