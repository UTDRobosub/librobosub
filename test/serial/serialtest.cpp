
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
		ser.writeStr(str); //write the contents of str to the buffer
		
		while(true){
			
			char c[11]; //allocate a buffer to store received data in
			int l = ser.readLen(c,10); //read up to 10 characters from the serial port
			c[l] = '\0'; //append null to the end of the string
			if(l>0){
				cout<<c; //print as a string
			}else{
				break;
			}
			//NB: readStr does pretty much the same thing, up to 8192 characters
		}
	}
	
	return 0;
}
