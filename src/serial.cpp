
#include "robosub/serial.h"

namespace robosub {

	////////////////////////////////////////////////////////////////////////////////////////////
	//Encoding and decoding; not part of Serial class.
	//Copy and paste this code into an arduino and it will still work.

	int strFindFirstMasked(char *buf, int maxlen, char until, char mask){
		for(int i=0; i<maxlen; i++){
			if((buf[i]&mask)==until){
				return i;
			}
		}

		return -1;
	}

	int strFindLastMasked(char *buf, int maxlen, char until, char mask){
		for(int i=maxlen-1; i>=0; i--){
			if((buf[i]&mask)==until){
				return i;
			}
		}
	}

	const char Serial_MsgBeginChar = '[';
	const char Serial_MsgEndChar = ']';
	const char Serial_EscapeChar = '\\';

	int Serial_SpecialCharCount = 3;
	const char Serial_SpecialChars[] = {
		Serial_MsgBeginChar,
		Serial_MsgEndChar,
		Serial_EscapeChar,
	};
	const char Serial_SpecialCharEscapes[] = {
		'(',
		')',
		'/',
	};

	int SerialDataEncode(char *decoded, int decodedlen, char *encoded){
		int encodedlen = 0;

		encoded[encodedlen++] = Serial_MsgBeginChar;

		for(int ci=0; ci<decodedlen; ci++){
			char c = decoded[ci];

			bool special = false;
			for(int scci=0; scci<Serial_SpecialCharCount; scci++){
				if(c==Serial_SpecialChars[scci]){
					encoded[encodedlen++] = Serial_EscapeChar;
					encoded[encodedlen++] = Serial_SpecialCharEscapes[scci];
					special = true;
				}
			}

			if(!special){
				encoded[encodedlen++] = c;
			}
		}

		encoded[encodedlen++] = Serial_MsgEndChar;

		return encodedlen;
	}

	int SerialDataDecode(char *encoded, int encodedlen, char *decoded){
		int decodedlen = 0;

		for(int ci=0; ci<encodedlen; ci++){
			char c = encoded[ci];

			if(c==Serial_EscapeChar){ //if the current character is the escape, the next character is an escaped special char
				ci++;
				c = encoded[ci];

				bool special = false;
				for(int scci=0; scci<Serial_SpecialCharCount; scci++){
					if(c==Serial_SpecialCharEscapes[scci]){
						decoded[decodedlen++] = Serial_SpecialChars[scci];
						special = true;
					}
				}

				if(!special){ //the last character was the escape but the current one does not match any special characters
					//cout<<"Error: Escaped non-special char "<<c<<endl;
				}
			}else{ //otherwise it's a regular character
				decoded[decodedlen++] = c;
			}
		}

		return decodedlen;
	}

	int SerialReadAndRemoveFirstEncodedDataFromBuffer(char *buf, int *buflen, char *decoded, int maxdecodedlen){
		int startloc = strFindFirstMasked(buf, *buflen, Serial_MsgBeginChar, 0xFF); //find the first start marker in the string
		int endloc = strFindFirstMasked(buf+startloc+1, *buflen-startloc-1, Serial_MsgEndChar, 0xFF);

		if(startloc!=-1 && endloc!=-1){
			endloc += startloc+1;

			startloc = strFindLastMasked(buf, endloc, Serial_MsgBeginChar, 0xFF);

			int encodedlen = endloc-startloc-1;
			char *encoded = (char*)malloc(encodedlen+1);

			memcpy(encoded, buf+startloc+1, encodedlen);

			memmove(buf, buf+endloc, *buflen-endloc-1);
			*buflen -= endloc+1;

			return SerialDataDecode(encoded, encodedlen, decoded);
		}

		return 0;
	}

	////////////////////////////////////////////////////////////////////////////////////////////

	//sorry about the flow, this is what happens when you have a possibility for errors in the constructor
	Serial::Serial(string fn, int baud){
		connected = false;

		filename = fn;
		readbuflen = 0;
		readbuf = (char*)malloc(maxreadbuflen);

		int ibaud = 0;
		switch(ibaud){
			case 0:       ibaud = B0;       break;
			case 50:      ibaud = B50;      break;
			case 75:      ibaud = B75;      break;
			case 110:     ibaud = B110;     break;
			case 134:     ibaud = B134;     break;
			case 150:     ibaud = B150;     break;
			case 200:     ibaud = B200;     break;
			case 300:     ibaud = B300;     break;
			case 600:     ibaud = B600;     break;
			case 1200:    ibaud = B1200;    break;
			case 1800:    ibaud = B1800;    break;
			case 2400:    ibaud = B2400;    break;
			case 4800:    ibaud = B4800;    break;
			case 9600:    ibaud = B9600;    break;
			case 19200:   ibaud = B19200;   break;
			case 38400:   ibaud = B38400;   break;
			case 57600:   ibaud = B57600;   break;
			case 115200:  ibaud = B115200;  break;
			case 230400:  ibaud = B230400;  break;
			case 460800:  ibaud = B460800;  break;
			case 500000:  ibaud = B500000;  break;
			case 576000:  ibaud = B576000;  break;
			case 921600:  ibaud = B921600;  break;
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
	}

	Serial::~Serial(){
		free(readbuf);
		close(file);
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

	//whether it was successfully initialized and is currently operational
	bool Serial::isConnected(){
		return connected;
	}

	//empty the buffer of unprocessed received data
	void Serial::flushBuffer(){
		readbuflen = 0;
	}

	//reads up to maxlen chars from the beginning of the receive buffer
	int Serial::readLen(char *buf, int maxlen){
		readEntireBuffer();

		int readlen = min(maxlen, readbuflen);

		if(buf!=nullptr){
			memcpy(buf, readbuf, readlen);
		}
		memmove(readbuf, readbuf+readlen, readbuflen-readlen);

		readbuflen -= readlen;

		return readlen;
	}

	//reads until it encounters until or has read maxlen chars
	int Serial::readToChar(char *buf, int maxlen, char until){
		return readToFlag(buf, maxlen, until, 0xFF);
	}

	//reads until it encounters a byte whose masked bits equal until, or it has read maxlen chars
	int Serial::readToFlag(char *buf, int maxlen, char until, char mask){
		int firstloc = strFindFirstMasked(readbuf, readbuflen, until, mask) + 1;
		if(firstloc==-1) return -2;
		else if(firstloc>maxlen) return -1;

		int rlen = readLen(buf, firstloc);

		return rlen;
	}

	//reads all available data into a c++ string; may discard data if the buffer contains null characters
	string Serial::readStr(){
		readEntireBuffer();
		readbuf[readbuflen] = '\0';

		readbuflen = 0;

		return (string)readbuf;
	}

	//transmits len chars out of buf
	void Serial::writeLen(char *buf, int len){
		if(!connected){
			cout<<"Serial port \'"<<filename<<"\' tried to write but not connected"<<endl;
			return;
		}

		write(file, buf, len);
	}

	//transmits the string
	void Serial::writeStr(string str){
		char *s = (char*)str.c_str();
		writeLen(s, str.length());
	}

	//reads and decodes the first available packet, discarding data after the packet is maxdecodedlen bytes in size
	int Serial::readDecodeLen(char *decoded, int maxdecodedlen){
		readEntireBuffer();

		return SerialReadAndRemoveFirstEncodedDataFromBuffer(readbuf, &readbuflen, decoded, maxdecodedlen);
	}

	//reads and decodes the first available packet into a string; may discard data if the packet contains nulls
	string Serial::readDecodeStr(){
		readEntireBuffer();

		char decoded[1024];

		int decodedlen = SerialReadAndRemoveFirstEncodedDataFromBuffer(readbuf, &readbuflen, decoded, 1024);

		decoded[decodedlen] = '\0';

		return (string)decoded;
	}

	//encodes and transmits decodedlen chars out of decoded
	void Serial::writeEncodeLen(char *decoded, int decodedlen){
		char encoded[decodedlen*2];

		int encodedlen = SerialDataEncode(decoded, decodedlen, encoded); //leave space at the beginning of encoded for the start character

		writeLen(encoded, encodedlen);
	}

	//encodes and transmits the string
	void Serial::writeEncodeStr(string str){
		char *s = (char*)str.c_str();
		writeEncodeLen(s, str.length());
	}
}
