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

static int getTrackbar(char *name) {
    return cv::getTrackbarPos(name, "Output");
}

void updateTrackbars(ShapeFinder &shapeFinder) {
    shapeFinder.MIN_AREA = (double) getTrackbar("MIN_AREA");
    shapeFinder.MAX_AREA = (double) getTrackbar("MAX_AREA");
    shapeFinder.EROSION_SIZE = getTrackbar("EROSION_SIZE");
    shapeFinder.SQUARE_RATIO_THRESHOLD = getTrackbar("SQUARE_RATIO_THRESHOLD") / 100.0;
    shapeFinder.TRIANGLE_RATIO_THRESHOLD = getTrackbar("TRIANGLE_RATIO_THRESHOLD") / 100.0;
    shapeFinder.EPSILON_APPROX_TOLERANCE_FACTOR = getTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR") / 1000.0;
    shapeFinder.IMAGE_BLACK_THRESHOLD = getTrackbar("IMAGE_BLACK_THRESHOLD") / 10.0;
    shapeFinder.CONTOUR_BLACK_THRESHOLD = getTrackbar("CONTOUR_BLACK_THRESHOLD") / 10.0;
}


void displayShapes(ShapeFindResult &result,
                   const Mat &outputImage) {
    drawContours(outputImage, result.triangles, -1, Scalar(213, 0, 249), 4, 8); //magenta
    drawContours(outputImage, result.rectangles, -1, Scalar(0, 176, 255), 4, 8); //blue
    drawContours(outputImage, result.squares, -1, Scalar(255, 255, 0), 4, 8); //yellow
    drawContours(outputImage, result.circles, -1, Scalar(100, 230, 0), 4, 8); //teal

}

void displayShapeCountUi(ShapeFindResult &result, Mat &outputImage) {
    vector<vector<Point>> tp = vector<vector<Point>>({{Point(80, 100), Point(120, 100), Point(100, 60)}});
    circle(outputImage, Point(100, 30), 20, Scalar(0, 0, 255), -1);
    fillPoly(outputImage, tp, Scalar(0, 0, 255));
    Drawing::rectangle(outputImage, Point(80, 130), Point(120, 135), Scalar(0, 0, 255),
                       Scalar(0, 0, 255));
    Drawing::rectangle(outputImage, Point(80, 165), Point(120, 205), Scalar(0, 0, 255),
                       Scalar(0, 0, 255));

    Drawing::text(outputImage, to_string(result.getCountMode(result.circleCounts)), Point(20, 50),
                  Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    Drawing::text(outputImage, to_string(result.getCountMode(result.triangleCounts)),
                  Point(20, 100), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    Drawing::text(outputImage, to_string(result.getCountMode(result.rectangleCounts)),
                  Point(20, 150), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    Drawing::text(outputImage, to_string(result.getCountMode(result.squareCounts)), Point(20, 200),
                  Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
}


static void makeTrackbar(char *name, int length) {
    int *v = new int(1);
    cv::createTrackbar(name, "Output", v, length);
}

static void createViewingWindows() {
    namedWindow("Output");
}

void createTuningWindow(ShapeFinder shapeFinder) {
    createViewingWindows();

    makeTrackbar("MIN_AREA", 10000);
    makeTrackbar("MAX_AREA", 10000);
    makeTrackbar("EROSION_SIZE", 20);
    makeTrackbar("SQUARE_RATIO_THRESHOLD", 100);
    makeTrackbar("TRIANGLE_RATIO_THRESHOLD", 100);
    makeTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR", 100);
    makeTrackbar("CONTOUR_BLACK_THRESHOLD", 10000);
    makeTrackbar("IMAGE_BLACK_THRESHOLD", 10000);


    setTrackbarPos("MIN_AREA", "Output", (int) shapeFinder.MIN_AREA);
    setTrackbarPos("MAX_AREA", "Output", (int) shapeFinder.MAX_AREA);
    setTrackbarPos("EROSION_SIZE", "Output", shapeFinder.EROSION_SIZE);
    setTrackbarPos("SQUARE_RATIO_THRESHOLD", "Output", (int) (shapeFinder.SQUARE_RATIO_THRESHOLD * 100));
    setTrackbarPos("TRIANGLE_RATIO_THRESHOLD", "Output", (int) (shapeFinder.TRIANGLE_RATIO_THRESHOLD * 100));
    setTrackbarPos("EPSILON_APPROX_TOLERANCE_FACTOR", "Output",
                   (int) (shapeFinder.EPSILON_APPROX_TOLERANCE_FACTOR * 1000));
    setTrackbarPos("IMAGE_BLACK_THRESHOLD", "Output", (int) (shapeFinder.IMAGE_BLACK_THRESHOLD * 10));
    setTrackbarPos("CONTOUR_BLACK_THRESHOLD", "Output", (int) (shapeFinder.CONTOUR_BLACK_THRESHOLD * 10));
}

void displayFrameCount(double rate, Mat &outputImage) {
    Drawing::text(outputImage, to_string(rate), Point(20, 50),
                  Scalar(255, 0, 0), Drawing::BOTTOM_LEFT, 2, 4);
}

int main(int argc, char **argv) {
    // TODO: Fix this to compile. The code should be correct, but the project build is having issues
    struct sigaction action = getSigaction();

    sigaction(SIGINT, &action, NULL);

    // cam.setFrameSize(Size(1280, 720));
    Camera::CalibrationData calibrationData = *Camera::loadCalibrationDataFromXML("../config/fisheye_cameracalib.xml",
                                                                                  std::move(Size(1280, 720)));
    Mat input, output;
    Camera cam = Camera("/dev/video0");
    ShapeFinder shapeFinder(calibrationData);
    ShapeFindResult result;

    createTuningWindow(shapeFinder);


    while (running) {
        cam.retrieveFrameBGR(input);
        updateTrackbars(shapeFinder);

        shapeFinder.processFrame(input, result);
        displayShapes(result, input);
        displayShapeCountUi(result, input);
        displayFrameCount(cam.getFrameRate(), input);

        ImageTransform::scale(input, .5);
        ImageTransform::scale(input, .5);

        imshow("Output", input);

        int keyPress = waitKey(1);

        if (keyPress == 32)
            waitKey(0);
        else if (keyPress >= 65 && keyPress <= 122)
            break;
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

