
#include <robosub/serial.h>
#include <robosub/util.h>

using namespace std;
using namespace robosub;

int main(){
	//cout<<"enter port (i.e. /dev/ttyACM0): ";
	//string port; //i.e. /dev/ttyACM0 or /dev/ttyUSB1
	//cin>>port;   //ls /dev | grep tty[AU]
	
	//Serial serial(port.c_str());
	
	string port = Util::execCLI("ls /dev | grep tty[AU]");
	Serial serial = Serial("/dev/" + port.substr(0,port.length()-1));
  
	while(true){
		char serdata[100];
		int serlen;
		
		serlen = serial.readLen(serdata, 99);
		
		serdata[serlen] = '\0';
		
		if(serlen<0){
			cout<<serdata<<endl;
		}
	}
	
	return 0;
}
