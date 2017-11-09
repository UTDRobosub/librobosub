#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>

using namespace std;
using namespace robosub;

//sample to test independent sending and receiving of strings, on local loopback
int main(){
	int e;

	int mode;
	int port;
	int port2;
	string addr;
	cout<<"Enter mode (0 for receive, 1 for send, 2 for bi-directional): ";
	cin>>mode;
	if(mode==2){
        cout<<"Enter port for receiving: ";
        cin>>port;
        cout<<"Enter port for sending: ";
        cin>>port2;
	}else{
        cout<<"Enter port: ";
        cin>>port;
	}
	if(mode==1 || mode==2){
        cout<<"Enter address (leave blank for receive mode): ";
        cin>>addr;
	}

    if(mode==0){
	    UDPR a;
        e=a.initRecv(port);
        cout<<"initRecv err: "<<e<<"\n";
		while(true){

			string msg;
			e=a.recvStr(msg);

			//if(msg!=""){
                cout<<"recv err: "<<e<<"\n";
                cout<<"recv msg: \""<<msg<<"\"\n";
			//}

		}
	}
	else if(mode==1){
        UDPS a;
        e=a.initSend(port,addr);
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
	}else if(mode==2){
        UDP a;
        e=a.init(port,port2,addr);
        cout<<"init err: "<<e<<"\n";

        while(true){
            string msg;
            cin>>msg;
            if(msg!=""){
                for(int i=0;i<msg.length(); i++){
                    if(msg[i]=='0')msg[i]='\0';
                }
                e=a.sendStr(msg);
                cout<<"send err: "<<e<<"\n";
            }

            string rmsg;
            e=a.recvStr(rmsg);
            cout<<"recv err: "<<e<<"\n";
            cout<<"recv msg: \""<<rmsg<<"\"\n";
        }
	}

	return 0;
}
