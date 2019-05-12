#pragma once

#include "common.h"

#ifdef WINDOWS
	#define NETWORKUDP_WINSOCK
#endif

#include <string.h>
#include <errno.h>

#ifdef NETWORKTCP_WINSOCK
    #include <WS2tcpip.h>
    #include <Winsock2.h>
#else
    #include <unistd.h>
    #include <arpa/inet.h>
#endif

#ifdef NETWORKTCP_WINSOCK
    #define NETWORKTCP_GETERROR WSAGetLastError()
#else
    #define NETWORKTCP_GETERROR errno
#endif

namespace robosub{
	
	///////////////////////////////////////////////////////////
	//Server
	
	class NetworkTcpServer{
		bool bound;
		bool connected;
		
		sockaddr_in addr;
		int server;
		int client;
		
		public:
		
		NetworkTcpServer(){
			
		}
		~NetworkTcpServer(){
			
		}
		
		int bindToPort(int port);
		int unbindFromPort();
		int acceptClient();
		int dropClient();
		int receiveBuffer(char* data, int maxlen);
		int sendBuffer(char* data, int datalen);
	};
	
	///////////////////////////////////////////////////////////
	//Client
	
	class NetworkTcpClient{
		bool connected = false;
		
		sockaddr_in addr;
		int sock;

		public:
		
		NetworkTcpClient(){
			
		}
		~NetworkTcpClient(){
			
		}
		
		int connectToServer(char* saddr, int port);
		int disconnectFromServer();
		int receiveBuffer(char* data, int maxlen, int& numread);
		int sendBuffer(char* data, int datalen);
	};
}
