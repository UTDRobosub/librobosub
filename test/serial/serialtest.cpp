
#include <robosub/serial.h>

using namespace std;
using namespace robosub;

int main(){
	cout<<"enter port (i.e. /dev/ttyACM0): ";
	string port; //i.e. /dev/ttyACM0 or /dev/ttyUSB1
	cin>>port;   //ls /dev | grep tty[AU]
	
	Serial ser(port.c_str());
	
	while(true){
		string str;
		cin>>str;
		ser.writeStr(str);
		
		while(true){
			string rv=ser.readEntireBuffer();
			cout<<rv;
			if(rv==""){
				break;
			}
		}
	}
	
	return 0;
}
