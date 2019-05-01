#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <signal.h>

using namespace std;
using namespace robosub;
bool running = true;

double EPSILON_APPROX_TOLERANCE_FACTOR = 0.0425;
double MIN_AREA = 450;
double MAX_AREA = 8220;
double SQUARE_RATIO_THRESHOLD = .72;
double TRIANGLE_RATIO_THRESHOLD = .22;
int EROSION_SIZE = 1;

// TODO: Tune these parameters. Current values are okayish
double IMAGE_BLACK_THRESHOLD = 38;
double CONTOUR_BLACK_THRESHOLD = 101.5;

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
//    makeTrackbar("MIN_AREA", 10000);
//    makeTrackbar("MAX_AREA", 10000);
//    makeTrackbar("EROSION_SIZE", 20);
//    makeTrackbar("SQUARE_RATIO_THRESHOLD", 100);
//    makeTrackbar("TRIANGLE_RATIO_THRESHOLD", 100);
//    makeTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR", 10000);
//    makeTrackbar("IMAGE_BLACK_THRESHOLD", 10000);
//    makeTrackbar("CONTOUR_BLACK_THRESHOLD", 10000);

    Camera cam = Camera(1);
//    cam.setFrameSize(Size(1280, 720));
    auto calibrationData = *cam.loadCalibrationDataFromXML("../config/fisheye_cameracalib.xml",
                                                           cam.getFrameSize());

    if (!cam.isOpen()) return -1;

    Mat input, output, processed_img, contour_mask;
    Scalar mu, sigma;
    Ptr<CLAHE> clahe = createCLAHE(4, Size(16, 16));

    while (running) {

        //update trackbars
//        MIN_AREA = (double)getTrackbar("MIN_AREA");
//        MAX_AREA = (double)getTrackbar("MAX_AREA");
//        EROSION_SIZE = getTrackbar("EROSION_SIZE");
//        SQUARE_RATIO_THRESHOLD = getTrackbar("SQUARE_RATIO_THRESHOLD")/100.0;
//        TRIANGLE_RATIO_THRESHOLD = getTrackbar("TRIANGLE_RATIO_THRESHOLD")/100.0;
//        EPSILON_APPROX_TOLERANCE_FACTOR = getTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR")/10000.0;
//        IMAGE_BLACK_THRESHOLD = getTrackbar("IMAGE_BLACK_THRESHOLD")/10.0;
//        CONTOUR_BLACK_THRESHOLD = getTrackbar("CONTOUR_BLACK_THRESHOLD")/10.0;


        cam.retrieveFrameBGR(input);
        //undistort
        input = cam.undistort(input, calibrationData);

        cvtColor(input, processed_img, cv::COLOR_BGR2GRAY);
//        clahe->apply(processed_img, output);
        threshold(processed_img, output, IMAGE_BLACK_THRESHOLD, 255, 0);
        cvtColor(output, output, cv::COLOR_GRAY2RGB);

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

        bool got_contour_mask = false;

        findContours(processed_img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_L1);

        for (unsigned int i = 0; i < contours.size(); i++) {
            vector<Point> approx;
//            if (hierarchy[i][3] < 0) continue;  //has parent, inner (hole) contour of a closed edge (looks good)
            Contour c = Contour(contours[i]);
            approxPolyDP(contours[i], approx, EPSILON_APPROX_TOLERANCE_FACTOR * c.arcLength(true), true);


//            if (!c.isClosed()) continue;
//            if (c.area() > MIN_AREA && c.area() < MAX_AREA) cout << c.area() << endl;
//            else continue;

            const Scalar &averageColor = c.averageColor(output);

            if (averageColor[0] > CONTOUR_BLACK_THRESHOLD)
                continue;

            cout << c.area() << endl;
            if (c.area() > MAX_AREA || c.area() < MIN_AREA)
                continue;

            if (approx.size() >= 5) {
                circles.push_back(approx);
            } else if (approx.size() == 4) {
                Rectangle r = Rectangle(approx);
                double dimensionRatio = r.height() / r.width();
                if (dimensionRatio > 1) dimensionRatio = 1 / dimensionRatio;

                if (dimensionRatio < SQUARE_RATIO_THRESHOLD)
                    rectangles.push_back(approx);
                else
                    squares.push_back(approx);
            } else if (approx.size() == 3) {
                Triangle t = Triangle(approx);
                double dimensionRatio = t.height() / t.width();

                if (dimensionRatio < TRIANGLE_RATIO_THRESHOLD)
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
        drawContours(output, circles, -1, Scalar(100, 230, 0), 4, 8); //teal

        Drawing::text(output, to_string(circles.size()), Point(20, 50), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
        Drawing::text(output, to_string(triangles.size()), Point(20, 100), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
        Drawing::text(output, to_string(rectangles.size()), Point(20, 150), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
        Drawing::text(output, to_string(squares.size()), Point(20, 200), Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);

        vector<vector<Point>> tp = vector<vector<Point>>({{Point(80, 100), Point(120, 100), Point(100, 60)}});
        circle(output, Point(100, 30), 20, Scalar(0, 0, 255), -1);
        fillPoly(output, tp, Scalar(0, 0, 255));
        Drawing::rectangle(output, Point(80, 130), Point(120, 135), Scalar(0, 0, 255), Scalar(0, 0, 255));
        Drawing::rectangle(output, Point(80, 165), Point(120, 205), Scalar(0, 0, 255), Scalar(0, 0, 255));

        //Run canny detection with +- 1 std dev of random values
//        threshold1 = mu.val[0] - 0.66 * sigma.val[0];
//        threshold2 = mu.val[0] + 1.33 * sigma.val[0];
//        Canny(output, output, threshold1, threshold2);

        //Draw FPS text
        Drawing::text(output,
                      String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
                      Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5);

        ImageTransform::scale(input, 0.5);
        ImageTransform::scale(output, 0.5);

        imshow("Input", input);
        imshow("Output", output);
        if (got_contour_mask)
            imshow("Contour Mask", contour_mask);
        if (waitKey(1) >= 0) break;
        cout << "frame " << std::setprecision(4) << " @ " << cam.getFrameRate() << " fps" << endl;
    }
    return 0;
}

