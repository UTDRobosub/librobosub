
#pragma once

#include "common.h"

#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

struct Serial_State;

namespace robosub {
	const int Serial_MaxBufferLen = 65536;
	
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
		
		Serial_State* state;
		bool useprotocol;
		
		bool debugPrint;
		
		void (*receiveMessageCallback)(char* message, int length, bool needsrepsonse, char** response, int* responsepength);
		
		void writeLen(char* buffer, int length);
		void readEntireBuffer();
		
		public:
		Serial(string fn, int baud, void (*receiveMessageCallback)(char* message, int length, bool needsrepsonse, char** response, int* responsepength), bool useprotocol, bool debugprint = false);
		~Serial();
		
		void onReceiveMessage(char* message, int length, bool needsrepsonse, char** response, int* responselength);
		
		void flushBuffer();
		
		void appendChar(char data);
		void transmitBuffer();
		
		bool isConnected();
		
		int receiveAllMessages();
		void transmitMessageFast(char* message, int length);
		void transmitMessageReliable(char* message, int length);
	};
}
