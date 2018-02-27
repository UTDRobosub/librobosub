
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
		
		public:
		Serial(string);
		~Serial();
		
		string readEntireBuffer();
		void writeOut(char*, int);
		void writeStr(string);
	};
}
