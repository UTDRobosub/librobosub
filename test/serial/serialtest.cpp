
#include <robosub/serial.h>
#include <robosub/util.h>

using namespace std;
using namespace robosub;

void printBits(char c){
	for(int j=7; j>=0; j--){
		bool bit = (c & (0b1<<j)) != 0;
		cout<<bit?'1':'0';
	}
	cout<<" "<<(int)c<<" \'"<<(char)c<<"\'"<<endl;
}

int main(){
	string port = Util::execCLI("ls /dev | grep tty[AU]");
	cout<<"using serial port /dev/"<<port<<endl;
	Serial serial1 = Serial("/dev/" + port.substr(0,port.length()-1), 115200);
	
	char decoded[1024];
	
	/*
	while(serial1.isConnected()){
		
		string sendstr;
		cin>>sendstr;
		
		serial1.writeEncodeStr(sendstr);
		
		string readstr = serial1.readDecodeStr();
		
		if(readstr.length()!=0){
			cout<<readstr<<endl;
		}
	}*/
	
	while(serial1.isConnected()){
		cout<<serial1.readDecodeLen(decoded, 1024)<<endl;
		
		int data = decoded[0] | decoded[1]<<8;
		
		cout<<data<<endl;
	}
	
	return 0;
}
