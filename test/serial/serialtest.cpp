
#include <robosub/serial.h>
#include <robosub/util.h>

using namespace std;
using namespace robosub;

void printBits(char c){
	for(int j=7; j>=0; j--){
		bool bit = (c & (0b1<<j)) != 0;
		cout<<bit?'1':'0';
	}
}

void serialDataConvert(char *serdata, int serlen){
	
	if((serlen>=2) && ((serdata[0]&0b11000000) == 0b00000000) && ((serdata[1]&0b11000000) == 0b01000000)){
		
		int explen = 0;
		explen |= (serdata[0]&0b00111111)<<6;
		explen |= (serdata[1]&0b00111111);
		
		if(explen == serlen){
			
			char recvdata[256];
			int recvlen = 0;
			
			for(int i=0; i<serlen-2; i++){
				char data = serdata[i+2];
				
				char recvbits;
				switch(i%4){
					case 0:
						recvbits =  (data&0b00111111)<<2;
						break;
					case 1:
						recvbits |= (data&0b00110000)>>4;
						recvdata[recvlen++] = recvbits;
						recvbits =  (data&0b00001111)<<4;
						break;
					case 2:
						recvbits |= (data&0b00111100)>>2;
						recvdata[recvlen++] = recvbits;
						recvbits =  (data&0b00000011)<<6;
						break;
					case 3:
						recvbits |= (data&0b00111111);
						recvdata[recvlen++] = recvbits;
						break;
				}
			}
			
			recvdata[recvlen] = '\0';
			
			cout<<recvlen<<endl;
			
			//cout<<recvdata<<endl;
			
			cout<<*(int*)(&recvdata[0])<<endl;
		}else{
			cout<<"invalid packet"<<endl;
		}
	}
}

int main(){
	//cout<<"enter port (i.e. /dev/ttyACM0): ";
	//string port; //i.e. /dev/ttyACM0 or /dev/ttyUSB1
	//cin>>port;   //ls /dev | grep tty[AU]
	
	//Serial serial(port.c_str());
	
	string port = Util::execCLI("ls /dev | grep tty[AU]");
	cout<<"using serial port /dev/"<<port<<endl;
	Serial serial = Serial("/dev/" + port.substr(0,port.length()-1));
	
	while(true){
		char serdata[256];
		int serlen;
		
		serlen = serial.readLen(serdata, 256);
		
		for(int i=0; i<serlen; i++){
			cout<<(int)serdata[i]<<endl;
		}
	}
	
	/*
	while(true){
		char serdata[256];
		int serlen;
		
		serlen = serial.readToFlag(serdata, 256, 0b11000000, 0b11000000);
		
		serialDataConvert(serdata, serlen);
	}
	
	*/
	
	return 0;
}
