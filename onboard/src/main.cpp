#include "main.h"
#include "robot.h"

using namespace std;
using namespace robosub;

bool running = true;

void server();
void startVideo();

int main(int argc, char** argv) {
    initRobotState();

    //start server thread
    thread serverThread(server);

    //start video system and wait for video threads to join
    startVideo();

    //wait for server thread to join
    serverThread.join();
    return 0;
}