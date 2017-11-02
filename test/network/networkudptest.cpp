#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>

using namespace std;
using namespace robosub;

//sample to test independent sending and receiving of strings, on local loopback
int main(){
	int e;

	int mode;
	cin>>mode;

	if(mode==1){
        UDPS a;
        e=a.initSend(5009,"localhost");
        cout<<"initSend err: "<<e<<"\n";
		while(true){

			string msg;
			cin>>msg;
            for(int i=0;i<msg.length(); i++){
                if(msg[i]=='0')msg[i]='\0';
            }
			e=a.sendStr(msg);

			cout<<"send err: "<<e<<"\n";

		}
	}else{
	    UDPR a;
        e=a.initRecv(5009);
        cout<<"initRecv err: "<<e<<"\n";
		while(true){

			string msg;
			e=a.recvStr(msg);

			if(msg!=""){
                cout<<"recv err: "<<e<<"\n";
                cout<<"recv msg: \""<<msg<<"\"\n";
			}

		}
	}

	return 0;
}
