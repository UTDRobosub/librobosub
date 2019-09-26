
#include <iostream>
#include "robosub/networktcp.h"

namespace robosub {

    ///////////////////////////////////////////////////////////
    //Server

    int NetworkTcpServer::bindToPort(int port) {
        if ((server = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
            return NETWORKTCP_GETERROR;
        }

        int opt = 1;
        if (setsockopt(server, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt)) != 0) {
            return NETWORKTCP_GETERROR;
        }

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);

        if (bind(server, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            return NETWORKTCP_GETERROR;
        }

        bound = true;
    }

    int NetworkTcpServer::unbindFromPort() {
        if (!bound) { return -1; }
        if (connected) { dropClient(); }
#ifdef NETWORKTCP_WINSOCK
        WSACleanup();
        closesocket(server);
#else
        close(server);
#endif

        bound = false;

        return 0;
    }

    int NetworkTcpServer::acceptClient() {
        if (listen(server, 3) < 0) {
            return NETWORKTCP_GETERROR;
        }

        int addrlen = sizeof(addr);
        if ((client = accept(server, (struct sockaddr *) &addr, (socklen_t *) &addrlen)) < 0) {
            return NETWORKTCP_GETERROR;
        }

        connected = true;
    }

    int NetworkTcpServer::dropClient() {
        if (!connected) { return -1; }
#ifdef NETWORKTCP_WINSOCK
        WSACleanup();
        closesocket(client);
#else
        close(client);
#endif

        connected = false;
    }

    int NetworkTcpServer::receiveBuffer(char *data, int maxlen) {
        if (!connected) { return -1; }
        return read(client, data, maxlen);
    }

    int NetworkTcpServer::sendBuffer(char *data, int datalen) {
        if (!connected) { return -1; }
        int numsent = send(client, data, datalen, 0);

        if (numsent == -1) {
            return NETWORKTCP_GETERROR;
        }

        return 0;
    }

    ///////////////////////////////////////////////////////////
    //Client

    int NetworkTcpClient::connectToServer(char *saddr, int port) {
        if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
            return NETWORKTCP_GETERROR;
        }

        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
//		addr.sin_addr.s_addr = inet_addr(saddr);
        addr.sin_port = htons(port);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *) &tv, sizeof tv);

        if (inet_pton(AF_INET, saddr, &addr.sin_addr) <= 0) {
            cout << "E INET_PTON" << endl;
            return NETWORKTCP_GETERROR;
        }

        cout << "TEST" << endl;

        if (connect(sock, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            cout << "E CONNECT" << endl;
            return NETWORKTCP_GETERROR;
        }

        connected = true;
        cout << "CONNECT OK" << endl;

        return 0;
    }

    int NetworkTcpClient::disconnectFromServer() {
#ifdef NETWORKTCP_WINSOCK
        WASCleanup();
        closesocket(sock);
#else
        close(sock);
#endif

        connected = false;
    }

    int NetworkTcpClient::receiveBuffer(char *data, int maxlen, int &numread) {
        if (!connected) { return -1; }
        numread = recv(sock, data, maxlen, 0);
        if (numread == -1) {
            int ecode = NETWORKTCP_GETERROR;
            if (ecode != 11) {
                return ecode;
            }
        }
        return 0;
    }

    int NetworkTcpClient::sendBuffer(char *data, int datalen) {
        if (!connected) { return -1; }
        send(sock, data, datalen, 0);

        return 0;
    }
}
