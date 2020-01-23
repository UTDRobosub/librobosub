//
// Created by robosub on 11/13/19.
//

#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <csignal>

using namespace std;
using namespace robosub;

bool RUNNING = true;
Camera::CalibrationData CALIBRATION_DATA;
TuningSample *CURRENT_SAMPLE;
int N_SQUARES, N_CIRCLES, N_RECTANGLES, N_TRIANGLES;
vector<TuningSample> *TUNING_SAMPLES;

struct sigaction getSigaction();

void catchSignal(int signal) {
    RUNNING = false;
}

static int getTrackbar(char *name) {
    return cv::getTrackbarPos(name, "Output");
}

static void updateSample(TuningSample *sample) {
    sample->sampleData->insert({"N_SQUARES", (void *) &N_SQUARES});
    sample->sampleData->insert({"N_CIRCLES", (void *) &N_CIRCLES});
    sample->sampleData->insert({"N_RECTANGLES", (void *) &N_RECTANGLES});
    sample->sampleData->insert({"N_TRIANGLES", (void *) &N_TRIANGLES});
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

    Drawing::text(outputImage, to_string(result.getCountMode(result.circleCounts)), Point(0, 50),
                  Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    Drawing::text(outputImage, to_string(result.getCountMode(result.triangleCounts)),
                  Point(0, 100), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    Drawing::text(outputImage, to_string(result.getCountMode(result.rectangleCounts)),
                  Point(0, 150), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    Drawing::text(outputImage, to_string(result.getCountMode(result.squareCounts)), Point(0, 200),
                  Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
}


static void createViewingWindows() {
    namedWindow("Output");
}

void createTuningWindow() {
    createViewingWindows();

    cv::createTrackbar("N_SQUARES", "Output", &N_SQUARES, 10);
    cv::createTrackbar("N_CIRCLES", "Output", &N_CIRCLES, 10);
    cv::createTrackbar("N_RECTANGLES", "Output", &N_RECTANGLES, 10);
    cv::createTrackbar("N_TRIANGLES", "Output", &N_TRIANGLES, 10);
}

void displayFrameCount(double rate, Mat &outputImage) {
    Drawing::text(outputImage, to_string(rate), Point(20, 50),
                  Scalar(255, 0, 0), Drawing::BOTTOM_LEFT, 2, 4);
}

void setParameters(ShapeFinder &sf, map<string, double> parameters) {
    sf.EPSILON_APPROX_TOLERANCE_FACTOR = parameters.at("EPSILON_APPROX_TOLERANCE_FACTOR");
    sf.MIN_AREA = parameters.at("MIN_AREA");
    sf.MAX_AREA = parameters.at("MAX_AREA");
    sf.SQUARE_RATIO_THRESHOLD = parameters.at("SQUARE_RATIO_THRESHOLD");
    sf.TRIANGLE_RATIO_THRESHOLD = parameters.at("TRIANGLE_RATIO_THRESHOLD");
    sf.IMAGE_BLACK_THRESHOLD = parameters.at("IMAGE_BLACK_THRESHOLD");
    sf.CONTOUR_BLACK_THRESHOLD = parameters.at("CONTOUR_BLACK_THRESHOLD");
}

double parameterEvaluationFunction(map<string, double> parameters) {
    ShapeFinder sf(CALIBRATION_DATA);
    setParameters(sf, parameters);

    double error = 0;
    for (auto sample: *TUNING_SAMPLES) {
        auto result = new ShapeFindResult();
        sf.processFrame(sample.image, *result);

        error += abs(result->getLastSquareCount() - *((int *) sample.sampleData->at("N_SQUARES")));
        error += abs(result->getLastCircleCount() - *((int *) sample.sampleData->at("N_CIRCLES")));
        error += abs(result->getLastRectangleCount() - *((int *) sample.sampleData->at("N_RECTANGLES")));
        error += abs(result->getLastTriangleCount() - *((int *) sample.sampleData->at("N_TRIANGLES")));
    }

    return error;
}

int main(int argc, char **argv) {
    struct sigaction action = getSigaction();

    sigaction(SIGINT, &action, NULL);

    // cam.setFrameSize(Size(1280, 720));
    CALIBRATION_DATA = *Camera::loadCalibrationDataFromXML(
            "../config/fisheye_cameracalib.xml",
            std::move(Size(1280, 720)));
    Mat raw, input, output;
    Camera cam = Camera("/dev/video0");
    ShapeFinder shapeFinder(CALIBRATION_DATA);
    TUNING_SAMPLES = new vector<TuningSample>();
    CURRENT_SAMPLE = new TuningSample();

    map<string, ParameterMetadata> parameters = map<string, ParameterMetadata>{
            {"EPSILON_APPROX_TOLERANCE_FACTOR", ParameterMetadata(shapeFinder.EPSILON_APPROX_TOLERANCE_FACTOR, 0, 1)},
            {"MIN_AREA",                        ParameterMetadata(shapeFinder.MIN_AREA, 0, 100)},
            {"MAX_AREA",                        ParameterMetadata(shapeFinder.MAX_AREA, 1000, 10000)},
            {"SQUARE_RATIO_THRESHOLD",          ParameterMetadata(shapeFinder.SQUARE_RATIO_THRESHOLD, 0, 1)},
            {"TRIANGLE_RATIO_THRESHOLD",        ParameterMetadata(shapeFinder.TRIANGLE_RATIO_THRESHOLD, 0, 1)},
            {"IMAGE_BLACK_THRESHOLD",           ParameterMetadata(shapeFinder.IMAGE_BLACK_THRESHOLD, 0, 300)},
            {"CONTOUR_BLACK_THRESHOLD",         ParameterMetadata(shapeFinder.CONTOUR_BLACK_THRESHOLD, 0, 300)}
    };


    ParameterTuner pt = ParameterTuner();
    ShapeFindResult result;

    createTuningWindow();


    while (RUNNING) {
        cam.retrieveFrameBGR(raw);
        raw.copyTo(input);
        imshow("Input", input);

        shapeFinder.processFrame(input, result);
        displayShapes(result, input);
        displayShapeCountUi(result, input);
        displayFrameCount(cam.getFrameRate(), input);

        ImageTransform::scale(input, .75);

        ImageTransform::scale(input, .75);

        imshow("Output", input);

        int keyPress = waitKey(1);

        if (keyPress == 32) { //take a picture and save values
            updateSample(CURRENT_SAMPLE);
            raw.copyTo(CURRENT_SAMPLE->image);

            TUNING_SAMPLES->push_back(*CURRENT_SAMPLE);
        } else if (keyPress == 84 || keyPress == 116) {
            auto bestParameters = pt.tuneParameters(parameters, parameterEvaluationFunction);

            setParameters(shapeFinder, bestParameters);

            cout << "Best parameters:" << endl;
            for (auto const &parameter: bestParameters) {
                cout << "\t" << parameter.first << ": " << parameter.second << endl;
            }
        } else if (keyPress >= 65 && keyPress <= 122)
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
