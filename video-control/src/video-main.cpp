#include <iostream>
#include <thread>
#include <robosub/robosub.h>
#include "video-main.h"

using namespace std;
//using namespace robosub;

const int NUMFEEDS = 3;
const int PORT[5] = {8500, 8501, 8502, 8503, 8504};
const String VIDEO_ADDR = "192.168.1.1";

void video();

bool running = true;
bool refresh = false;
mutex drawLock;

int main(int argc, char* argv[]){
    
    thread videoThread(video);
    videoThread.join();

    return 0;
}