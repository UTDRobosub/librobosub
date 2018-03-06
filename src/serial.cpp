
#include "robosub/serial.h"

namespace robosub {
	Serial::Serial(string fn, int baud){
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
		}
		
		memset(&tty, 0, sizeof(tty));
		if(tcgetattr(file, &tty)!=0){
			cout<<"Serial port \'"<<filename<<"\' tty tcgetattr error "<<errno<<endl;
		}
		
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
		}
	}
	
	Serial::~Serial(){
		free(readbuf);
		close(file);
	}
	 
	void Serial::flushBuffer(){
		readbuflen = 0;
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
	
	int strFindFirstFlag(char *buf, int maxlen, char until, char mask){
		for(int i=0; i<maxlen; i++){
			if((buf[i]&mask)==until){
				return i;
			}
		}
		
		return -1;
	}
	
	int Serial::readToChar(char *buf, int maxlen, char until){
		return readToFlag(buf, maxlen, until, 0xFF);
	}
	
	int Serial::readToFlag(char *buf, int maxlen, char until, char mask){
		readEntireBuffer();
		
		int firstloc = strFindFirstFlag(readbuf, readbuflen, until, mask) + 1;
		if(firstloc==-1) return -2;
		else if(firstloc>maxlen) return -1;
		
		int rlen = readLen(buf, firstloc);
		
		return rlen;
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
