#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <signal.h>

using namespace std;
using namespace robosub;

bool running = true;
double EPSILON_APPROX_TOLERANCE_FACTOR = 0.035;
double MIN_AREA = 100;
double MAX_AREA = 4500;
int EROSION_SIZE = 1;

void makeTrackbar(char *name, int length) {
    int *v = new int(1);
    cv::createTrackbar(name, "Output", v, length);
}

int getTrackbar(char *name) {
    return cv::getTrackbarPos(name, "Output");
}

void catchSignal(int signal) {
    running = false;
}

int main(int argc, char **argv) {
    //catch signal
    signal(SIGINT, catchSignal);

    namedWindow("Input");
    namedWindow("Output");

    //create trackbars
//    makeTrackbar("Min Area", 10000);
//    makeTrackbar("Max Area", 10000);
//    makeTrackbar("Erosion Size", 20);

    Camera cam = Camera(0);
//    auto calibrationData = *cam.loadCalibrationDataFromXML("../config/seawit_cameracalib.xml",
//                                                           cam.getFrameSize());

    if (!cam.isOpen()) return -1;

    Mat input, output, processed_img;
    Scalar mu, sigma;

    while (running) {

        //update trackbars
//        MIN_AREA = (double)getTrackbar("Min Area");
//        MAX_AREA = (double)getTrackbar("Max Area");
//        EROSION_SIZE = getTrackbar("Erosion Size");


        cam.retrieveFrameBGR(input);
        //undistort
//        input = cam.undistort(input, calibrationData);

        cvtColor(input, processed_img, cv::COLOR_BGR2GRAY);
        cvtColor(processed_img, output, cv::COLOR_GRAY2RGB);

        //Compute standard deviation for image
        meanStdDev(processed_img, mu, sigma);

        //Remove small noise
        Mat element = getStructuringElement(MORPH_RECT,
                                            Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1),
                                            Point(EROSION_SIZE, EROSION_SIZE));

        ImageFilter::downsample(processed_img, 2);
        ImageFilter::upsample(processed_img, 2);

        //Threshold
        //adaptiveThreshold(output, output, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 15, 8);
        double threshold1 = mu.val[0] - 2.0 * sigma.val[0];
        double threshold2 = mu.val[0] + 0.0 * sigma.val[0];
        Canny(processed_img, processed_img, threshold1, threshold2);


        dilate(processed_img, processed_img, element);
        erode(processed_img, processed_img, element);
        dilate(processed_img, processed_img, element);

//        bitwise_not(processed_img, processed_img);
        //Find contours
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        vector<vector<Point>> triangles;
        vector<vector<Point>> rectangles;
        vector<vector<Point>> squares;
        vector<vector<Point>> circles;

        findContours(processed_img, contours, hierarchy, RETR_TREE, CHAIN_APPROX_TC89_KCOS);
        for (unsigned int i = 0; i < contours.size(); i++) {
            vector<Point> approx;
            if (hierarchy[i][3] < 0) continue;  //has parent, inner (hole) contour of a closed edge (looks good)
            Contour c = Contour(contours[i]);
            approxPolyDP(contours[i], approx, EPSILON_APPROX_TOLERANCE_FACTOR * c.arcLength(true), true);


//            if (!c.isClosed()) continue;
//            if (c.area() > MIN_AREA && c.area() < MAX_AREA) cout << c.area() << endl;
//            else continue;

            if (approx.size() >= 5) {
                circles.push_back(approx);
            } else if (approx.size() == 4) {
                //TODO figure out how to calculate width/height ratio
                Rectangle r = Rectangle(approx);
                double dimensionRatio = r.height() / r.width();
                if (dimensionRatio > 1) dimensionRatio = 1 / dimensionRatio;

                cout << r.height() << " " << r.width() << endl;
                double ratioThreshold = .5;
                if (dimensionRatio < ratioThreshold)
                    rectangles.push_back(approx);
                else
                    squares.push_back(approx);
            } else if (approx.size() == 3) {
                Triangle t = Triangle(approx);
                double dimensionRatio = t.height() / t.width();

                double ratioThreshold = .5;
                if (dimensionRatio < ratioThreshold)
                    rectangles.push_back(approx);
                else
                    triangles.push_back(approx);
            } else {
                rectangles.push_back(approx);
            }
        }

        drawContours(output, triangles, -1, Scalar(213, 0, 249), 4, 8); //magenta
        drawContours(output, rectangles, -1, Scalar(0, 176, 255), 4, 8); //blue
        drawContours(output, squares, -1, Scalar(255, 255, 0), 4, 8); //yellow
        drawContours(output, circles, -1, Scalar(0, 230, 118), 4, 8); //green

        //Run canny detection with +- 1 std dev of random values
//        threshold1 = mu.val[0] - 0.66 * sigma.val[0];
//        threshold2 = mu.val[0] + 1.33 * sigma.val[0];
//        Canny(output, output, threshold1, threshold2);

        //Draw FPS text
        Drawing::text(output,
                      String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
                      Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5);

        ImageTransform::scale(processed_img, 2);
        ImageTransform::scale(output, 2);

        imshow("Input", processed_img);
        imshow("Output", output);
        if (waitKey(1) >= 0) break;
        cout << "frame " << std::setprecision(4) << " @ " << cam.getFrameRate() << " fps" << endl;
    }
    return 0;
}

