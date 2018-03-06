
#include "robosub/serial.h"

namespace robosub {
	Serial::Serial(string fn){
		filename = fn;
		readbuflen = 0;
		readbuf = (char*)malloc(maxreadbuflen);
		
		file = open(filename.c_str(), O_RDWR | O_NOCTTY);
		if(file<0){
			cout<<"Serial port \'"<<filename<<"\' failed to open: error "<<errno<<endl;
		}
		
		memset(&tty, 0, sizeof(tty));
		if(tcgetattr(file, &tty)!=0){
			cout<<"Serial port \'"<<filename<<"\' tty tcgetattr error "<<errno<<endl;
		}
		
		cfsetospeed(&tty, B9600);
		cfsetispeed(&tty, B9600);
		
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
		}
	}
	
	Serial::~Serial(){
		free(readbuf);
		close(file);
	}
	
	void Serial::readEntireBuffer(){
		readbuflen += read(file, readbuf+readbuflen, maxreadbuflen-readbuflen);
		
		if(readbuflen==maxreadbuflen){
			cout<<"Serial port \'"<<filename<<"\' read buffer full"<<endl;
		}
	}
	
	int Serial::readLen(char *buf, int maxlen){
		readEntireBuffer();
		
		int readlen = min(maxlen, readbuflen);
		
		memcpy(buf, readbuf, readlen);
		memmove(readbuf, readbuf+readlen, readbuflen-readlen);
		
		readbuflen -= readlen;
		
		return readlen;
	}
	
	int Serial::readToNull(char *buf, int maxlen){
		readEntireBuffer();
		
		int firstnull=0;
		for(int i=0; i<readbuflen; i++){
			if(readbuf[i]=='\0'){
				firstnull=i;
				break;
			}
		}
		
		int readlen = min(maxlen, firstnull);
		
		memcpy(buf, readbuf, readlen);
		memmove(readbuf, readbuf+firstnull, readbuflen-firstnull);
		
		readbuflen -= firstnull;
		
		return readlen;
	}
	
	string Serial::readStr(){
		readEntireBuffer();
		readbuf[readbuflen] = '\0';
		
		readbuflen = 0;
		
		return (string)readbuf;
	}
	
	void Serial::writeLen(char *buf, int len){
		write(file, buf, len);
	}
	
	void Serial::writeStr(string str){
		char *s = (char*)str.c_str();
		writeLen(s, str.length());
	}
}
