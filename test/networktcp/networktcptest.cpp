
#include <opencv2/opencv.hpp>
#include <robosub/networktcp.h>
#include <robosub/timeutil.h>

using namespace std;
using namespace robosub;

const int MODE_CLIENT = 1;
const int MODE_SERVER = 2;

//sample to test independent sending and receiving of strings, on local loopback
int main() {
    int e;

    int mode;
    int port;
    string addr;

    cout << "Enter mode (1 for client, 2 for server): ";
    cin >> mode;

    if (mode == MODE_CLIENT) {
        cout << "Enter port to connect to: ";
        cin >> port;
    } else if (mode == MODE_SERVER) {
        cout << "Enter port to listen on: ";
        cin >> port;
    } else {
        cout << "Invalid mode." << endl;
        return 0;
    }

    if (mode == MODE_CLIENT) {
        cin.ignore(100000, '\n');
        cout << "Enter address to connect to (leave blank for local): ";
        if (cin.peek() != '\n') {
            cin >> addr;
            cin.ignore(100000, '\n');
        } else {
            addr = "127.0.0.1";
            cout << addr << endl;
        }
    }

    char sendbuffer[256];
    char sendbuflen = 0;

    char recvbuffer[256];
    char recvbuflen = 0;

    if (mode == MODE_CLIENT) {
        NetworkTcpClient client;

        cout << "Connecting to server." << endl;
        client.connectToServer((char *) addr.c_str(), port);

        cout << "Connected." << endl;

        while (true) {
            Time::waitMillis(10);

            int recvbuflen = -1;
            int ecode = client.receiveBuffer(recvbuffer, 255, recvbuflen);
            if (ecode != 0) {
                cout << "BAD " << strerror(ecode) << endl;
            }
            recvbuffer[recvbuflen] = 0;

            cout << recvbuffer << endl;
        }
    } else if (mode == MODE_SERVER) {
        NetworkTcpServer server;

        cout << "Binding to port." << endl;
        server.bindToPort(port);

        cout << "Accepting client." << endl;
        server.acceptClient();

        cout << "Connected." << endl;

        while (true) {
            string send;
            cin >> send;

            server.sendBuffer((char *) send.c_str(), send.length());
        }
    }

    return 0;
}
