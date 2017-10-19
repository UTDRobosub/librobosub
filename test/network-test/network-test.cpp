
#include <string>
#include <iostream>
#include <string.h>
#include <unistd.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netdb.h>
#include <arpa/inet.h>

using namespace std;

//class for receiving
//call initRecv on the port to listen on before calling recv
//do not use multiple instances on the same port on one device, as receiving requires binding to the port
class UDPR{
	sockaddr_in raddr; //address info to recv on
	int rsock; //socket info for recving
	int initrecv; //1 if initRecv has succeeded

	public:

	UDPR(){
		initrecv=0;
	}
	UDPR(int p){
        initRecv(p);
    }
	~UDPR(){
        if(initrecv)stopRecv();
	}

	int initRecv(int);
	int stopRecv();
	int recv(string&);
};

//class for sending
//call initSend on the port and address to send to before calling send
class UDPS{
	sockaddr_in saddr; //address info to send on
	int ssock; //socket info for sending
	int initsend; //1 if initSend has succeeded

	public:

	UDPS(){
		initsend=0;
	}
	UDPS(int p,string a){
        initSend(p,a);
	}
	~UDPS(){
        if(initsend)stopSend();
	}

	int initSend(int,string);
	int stopSend();
	int send(string);
};

//wrapper class for bidirectional communication
//call init, which does both initSend and initRecv on the same port and the given address, before calling recv or send
//do not use multiple instances on the same port on one device, as receiving requires binding to the port
class UDP{
    UDPR ur;
    UDPS us;

    public:

    int init(int,string);
    int recv(string&);
    int send(string);
};

//set-up receiving on the specified port
//since it binds to the port, only one instance can receive on the same port on any device
//because of this, two bidirectional instances cannot be used on the same device on the same port
int UDPR::initRecv(int port){
	if((rsock=socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		return 2;
	}

	memset((char*)&raddr, 0, sizeof(raddr));
    raddr.sin_family=AF_INET;
    raddr.sin_addr.s_addr=htonl(INADDR_ANY);
    raddr.sin_port=htons(port);

    //bind to port; tell OS to send all incoming messages on this port to this instance
    //will err if already bound by another instance
    if(bind(rsock, (struct sockaddr*)&raddr, sizeof(raddr)) < 0){
        return 4;
    }

    initrecv=1;
    return 0;
}

//closes the socket
//should unbind the port and allow it to be bound again, but the OS can take several minutes to actually unbind the port after doing this
int UDPR::stopRecv(){
    if(!initrecv){
        return 128;
    }
    close(rsock);
    initrecv=0;

    return 0;
}

//reads up to 4095 characters from the receive queue and replaces the provided std::string with them
//does not wait for a message; will read 0 characters
int UDPR::recv(string &msg){
	if(!initrecv){
		return 32;
	}

	char buffer[4096];

	socklen_t addrlen=sizeof(raddr);

	int rlen;
	if((rlen=recvfrom(rsock, buffer, 4095, 0, (struct sockaddr*)&raddr, &addrlen)) < 0){
		return 64;
	}

    buffer[rlen]='\0'; //add null terminator
    msg=buffer; //copy null-terminated char array into return string

    return 0;
}

//set-up sending to the address in string form, on the specified port
//any amount of instances can send from or to any device
int UDPS::initSend(int port, string ssaddr){
	if((ssock=socket(AF_INET, SOCK_DGRAM, 0)) < 0){
		return 1;
	}

	memset((char*)&saddr, 0, sizeof(saddr));
    saddr.sin_family=AF_INET;
    inet_pton(AF_INET, ssaddr.c_str(), &saddr.sin_addr.s_addr);
    saddr.sin_port=htons(port);

    initsend=1;
    return 0;
}

//closes the socket
//basically a formality since it doesn't bind
int UDPS::stopSend(){
    if(!initsend){
        return 256;
    }
    close(ssock);
    initsend=0;

    return 0;
}

//transmits a std::string of length <= 4095
int UDPS::send(string msg){
	if(!initsend){
		return 8;
	}

	char buffer[4096];
    memcpy((void*)&(buffer[0]), (void*)msg.c_str(), msg.size()); //copy string contents into char array

	if(sendto(ssock, buffer, msg.length(), 0, (struct sockaddr*)&saddr, sizeof(saddr)) < 0){
		return 16;
	}

	return 0;
}

//wrapper functions for bidirectional communication
//see comments on their monodirectional counterparts

int UDP::init(int port, string ssaddr){
    return ur.initRecv(port) + us.initSend(port,ssaddr);
}

int UDP::recv(string &msg){
    return ur.recv(msg);
}

int UDP::send(string msg){
    return us.send(msg);
}

//sample to test independent sending and receiving on local loopback
int main(){
	int e;

	int mode;
	cin>>mode;

	if(mode==1){
        UDPS a;
        e=a.initSend(5007,"localhost");
        cout<<"initSend err: "<<e<<"\n";
		while(true){

			string msg;
			cin>>msg;
			e=a.send(msg);

			cout<<"send err: "<<e<<"\n";

		}
	}else{
	    UDPR a;
        e=a.initRecv(5007);
        cout<<"initRecv err: "<<e<<"\n";
		while(true){

			string msg;
			e=a.recv(msg);

			if(msg!=""){
                cout<<"recv err: "<<e<<"\n";
                cout<<"recv msg: \""<<msg<<"\"\n";
			}

		}
	}

	return 0;
}
