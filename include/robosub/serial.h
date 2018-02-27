
#pragma once

#include "common.h"

#include <cstring>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

namespace robosub {
	class Serial{
		string filename;
		
		int file;
		struct termios tty;
		
		char *readbuf;
		int readbuflen;
		const int maxreadbuflen = 8192;
		
		void readEntireBuffer();
		
		public:
		Serial(string);
		~Serial();
		
		int readLen(char*, int);
		string readStr();
		void writeLen(char*, int);
		void writeStr(string);
	};
}
