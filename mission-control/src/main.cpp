
#include <SDL/SDL.h>
#include <iostream>
#include <thread>
#include <robosub/robosub.h>
#include "main.h"

using namespace std;
using namespace robosub;

bool running = true;
bool refresh = false;

long controllerTime;
mutex drawLock;

ReadoutData readoutData;

int main(int argc, char *argv[]) {

    const String keys =
            "{help ?   |                    | print this message     }"
            "{f nfed   | 2                  | number of feeds        }"
            "{p port   | 8500               | starting port number   }"
            "{x pfix   |                    | file prefix            }"
            "{h host   | 192.168.1.1:8081   | network host           }";

    CommandLineParser parser(argc, argv, keys);
    parser.about("Network Video Transfer Test");

    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    int numFeeds = parser.get<int>("nfed");
    int port = parser.get<int>("port");
    String filePrefix = parser.get<String>("pfix");
    String networkHost = parser.get<String>("host");

    Controller* controller1 = new Controller;
    Controller* controller2 = new Controller;

    ThreadData* threaddata = new ThreadData;
    threaddata->readout = &readoutData;
    threaddata->networkFeeds = numFeeds;
    threaddata->port = port;
    threaddata->filePrefix = filePrefix;
    threaddata->networkHost = networkHost;
    threaddata->controller1 = controller1;
    threaddata->controller2 = controller2;



    thread controlThread(control, threaddata);
    thread networkThread(network, threaddata);
    thread readoutThread(readout, threaddata);

    controlThread.join();
    networkThread.join();
    readoutThread.join();

    delete controller1;
    delete controller2;
    delete threaddata;

    return 0;
}

//Thread for mission control window (everything but video feed)
//Display and send controller inputs,
//Recieve and display diagnostics
