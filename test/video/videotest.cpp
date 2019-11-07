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

static void makeTrackbar(char *name, int length) {
    int *v = new int(1);
    cv::createTrackbar(name, "Output", v, length);
}

static void createViewingWindows() {
    namedWindow("Input");
    namedWindow("Output");
}

void CreateTuningWindow(ShapeFinder shapeFinder) {
    createViewingWindows();

    makeTrackbar("MIN_AREA", 10000);
    makeTrackbar("MAX_AREA", 10000);
    makeTrackbar("EROSION_SIZE", 20);
    makeTrackbar("SQUARE_RATIO_THRESHOLD", 100);
    makeTrackbar("TRIANGLE_RATIO_THRESHOLD", 100);
    makeTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR", 100);
    makeTrackbar("CONTOUR_BLACK_THRESHOLD", 10000);
    makeTrackbar("IMAGE_BLACK_THRESHOLD", 10000);


    setTrackbarPos("MIN_AREA", "Output", (int) shapeFinder.MIN_AREA;
    setTrackbarPos("MAX_AREA", "Output", (int) shapeFinder.MAX_AREA;
    setTrackbarPos("EROSION_SIZE", "Output", shapeFinder.EROSION_SIZE);
    setTrackbarPos("SQUARE_RATIO_THRESHOLD", "Output", (int) (shapeFinder.SQUARE_RATIO_THRESHOLD * 100));
    setTrackbarPos("TRIANGLE_RATIO_THRESHOLD", "Output", (int) (shapeFinder.TRIANGLE_RATIO_THRESHOLD * 100));
    setTrackbarPos("EPSILON_APPROX_TOLERANCE_FACTOR", "Output",
                   (int) (shapeFinder.EPSILON_APPROX_TOLERANCE_FACTOR * 1000));
    setTrackbarPos("IMAGE_BLACK_THRESHOLD", "Output", (int) (shapeFinder.IMAGE_BLACK_THRESHOLD * 10));
    setTrackbarPos("CONTOUR_BLACK_THRESHOLD", "Output", (int) (shapeFinder.CONTOUR_BLACK_THRESHOLD * 10));
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

