#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <csignal>

using namespace std;
using namespace robosub;

bool running = true;

struct sigaction getSigaction();

void catchSignal(int signal) {
    running = false;
}

int main(int argc, char **argv) {
    // TODO: Fix this to compile. The code should be correct, but the project build is having issues
    struct sigaction action = getSigaction();

    sigaction(SIGINT, &action, NULL);

    Mat input;
    Camera cam = Camera("/dev/video0");
    ShapeFinder shapeFinder(cv::Size(1280, 720));

    while (running) {
        cam.retrieveFrameBGR(input);
        shapeFinder.processFrame(input);
    }

    return 0;
}

struct sigaction getSigaction() {
    struct sigaction action;
    sigemptyset(&action.sa_mask);
    action.sa_flags = 0;
    action.sa_handler = catchSignal;
    return action;
}

