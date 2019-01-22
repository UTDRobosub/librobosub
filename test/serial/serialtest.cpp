
#include <robosub/serial.h>
#include <robosub/util.h>
#include <robosub/timeutil.h>

using namespace std;
using namespace robosub;

void receiveMessage(char* message, int length, bool needsresponse, char** response, int* responselength){
	/*cout<<"Received: \"";
	for(int i=0; i<length; i++){
		cout<<(message[i]);
	}
    cout<<"\""<<endl;*/

	message[length] = 0;
	cout<<message<<endl;
}

int main(){
	string port = Util::execCLI("ls /dev | grep tty[AU]");
	cout<<"using serial port /dev/"<<port<<endl;
	Serial serial1 = Serial("/dev/" + port.substr(0,port.length()-1), 115200, receiveMessage, false);

	char decoded[1024];
	
	while(serial1.isConnected()){
		string valuestr;
		//cin>>valuestr;
		
		//int value = atoi(valuestr.c_str());

		int value = 0;
		if(value!=0){
			
			unsigned char senddata[3];
			senddata[0] = (unsigned char)((value&0x00FF) >> 0);
			senddata[1] = (unsigned char)((value&0xFF00) >> 8);
			senddata[2] = (unsigned char)(0                  );
			
			cout<<value<<endl;
			cout<<(int)senddata[0]<<" "<<(int)senddata[1]<<endl;

			serial1.transmitMessageReliable((char*)senddata, 2);
		}

		serial1.receiveAllMessages();

		robosub::Time::waitMillis(1);
	}

	return 0;
}
