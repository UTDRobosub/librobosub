
#pragma once

#include "common.h"

#include <string>
#include <iostream>
#include <string.h>
#include <unistd.h>
//#include <sys/types.h>
//#include <sys/socket.h>
//#include <netdb.h>
#include <arpa/inet.h>

namespace robosub{

	//class for receiving
	//call initRecv on the port to listen on before calling recv
	//do not use multiple instances on the same port on one device, as receiving requires binding to the port
	class UDPR{
		sockaddr_in raddr; //address info to recv on
		int rsock; //socket info for recving
		int initrecv; //1 if initRecv has succeeded

		public:

		UDPR();
		~UDPR();

		int initRecv(int);
		int stopRecv();
		int recv(int,int&,char*);
		int recvStr(string&);
	};

	//class for sending
	//call initSend on the port and address to send to before calling send
	class UDPS{
		sockaddr_in saddr; //address info to send on
		int ssock; //socket info for sending
		int initsend; //1 if initSend has succeeded

		public:

	    UDPS();
	    ~UDPS();

		int initSend(int,string);
		int stopSend();
		int send(int,char*);
		int sendStr(string);
	};

	//wrapper class for bidirectional communication
	//call init, which does both initSend and initRecv on the same port and the given address, before calling recv or send
	//do not use multiple instances on the same port on one device, as receiving requires binding to the port
	class UDP{
	    UDPR ur;
	    UDPS us;

	    public:

	    int init(int,string);
	    int recv(int,int&,char*);
	    int recvStr(string&);
	    int send(int,char*);
	    int sendStr(string);
	};

	//wrapper functions for bidirectional communication
	//for usage, see comments on their monodirectional counterparts
	inline int UDP::init(int port, string ssaddr){
	    return ur.initRecv(port) + us.initSend(port,ssaddr);
	}

	inline int UDP::recvStr(string &msg){
	    return ur.recvStr(msg);
	}
	inline int UDP::recv(int mlen, int &len, char *msg){
	    return ur.recv(mlen,len,msg);
	}

	inline int UDP::sendStr(string msg){
	    return us.sendStr(msg);
	}
	inline int UDP::send(int len, char *msg){
	    return us.send(len,msg);
	}
}
