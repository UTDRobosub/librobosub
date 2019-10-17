//
// Created by ryan on 10/9/19.
//

#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <utility>
#include "robosub/image-processing/shape_recognition.h"

using namespace std;

namespace robosub {
    static void makeTrackbar(char *name, int length) {
        int *v = new int(1);
        cv::createTrackbar(name, "Output", v, length);
    }

    static int getTrackbar(char *name) {
        return cv::getTrackbarPos(name, "Output");
    }

    static int mode(deque<int> const &q) {
        std::unordered_map<int, int> table;
        for (int i: q)
            table[i]++;

        int mode = 0;
        int mode_freq = 0;
        for (auto &iterator : table) {
            if (iterator.second > mode_freq) {
                mode = iterator.first;
                mode_freq = iterator.second;
            }
        }

        return mode;
    }

    static int getCountMode(deque<int> previousCounts, int nextCount, int lookBack) {
        if (previousCounts.empty()) {
            previousCounts.push_back(nextCount);
            return nextCount;
        } else if (previousCounts.size() < lookBack) {
            previousCounts.push_back(nextCount);
            return mode(previousCounts);
        } else {
            previousCounts.push_back(nextCount);
            previousCounts.pop_front();
            return mode(previousCounts);
        }
    }

    static void createViewingWindows() {
        namedWindow("Input");
        namedWindow("Output");
    }

    void ShapeFinder::createTuningWindow() {
        createViewingWindows();

        //create trackbars
        makeTrackbar("MIN_AREA", 10000);
        makeTrackbar("MAX_AREA", 10000);
        makeTrackbar("EROSION_SIZE", 20);
        makeTrackbar("SQUARE_RATIO_THRESHOLD", 100);
        makeTrackbar("TRIANGLE_RATIO_THRESHOLD", 100);
        makeTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR", 100);
        makeTrackbar("IMAGE_BLACK_THRESHOLD", 10000);
        makeTrackbar("CONTOUR_BLACK_THRESHOLD", 10000);

        setTrackbarPos("MIN_AREA", "Output", (int) MIN_AREA);
        setTrackbarPos("MAX_AREA", "Output", (int) MAX_AREA);
        setTrackbarPos("EROSION_SIZE", "Output", EROSION_SIZE);
        setTrackbarPos("SQUARE_RATIO_THRESHOLD", "Output", (int) (SQUARE_RATIO_THRESHOLD * 100));
        setTrackbarPos("TRIANGLE_RATIO_THRESHOLD", "Output", (int) (TRIANGLE_RATIO_THRESHOLD * 100));
        setTrackbarPos("EPSILON_APPROX_TOLERANCE_FACTOR", "Output", (int) (EPSILON_APPROX_TOLERANCE_FACTOR * 1000));
        setTrackbarPos("IMAGE_BLACK_THRESHOLD", "Output", (int) (IMAGE_BLACK_THRESHOLD * 10));
        setTrackbarPos("CONTOUR_BLACK_THRESHOLD", "Output", (int) (CONTOUR_BLACK_THRESHOLD * 10));
    }

    void ShapeFinder::updateTrackbars() {
        MIN_AREA = (double) getTrackbar("MIN_AREA");
        MAX_AREA = (double) getTrackbar("MAX_AREA");
        EROSION_SIZE = getTrackbar("EROSION_SIZE");
        SQUARE_RATIO_THRESHOLD = getTrackbar("SQUARE_RATIO_THRESHOLD") / 100.0;
        TRIANGLE_RATIO_THRESHOLD = getTrackbar("TRIANGLE_RATIO_THRESHOLD") / 100.0;
        EPSILON_APPROX_TOLERANCE_FACTOR = getTrackbar("EPSILON_APPROX_TOLERANCE_FACTOR") / 1000.0;
        IMAGE_BLACK_THRESHOLD = getTrackbar("IMAGE_BLACK_THRESHOLD") / 10.0;
        CONTOUR_BLACK_THRESHOLD = getTrackbar("CONTOUR_BLACK_THRESHOLD") / 10.0;
    }

    void
    ShapeFinder::classifyShape(vector<vector<Point>> &triangles, vector<vector<Point>> &rectangles,
                               vector<vector<Point>> &squares,
                               vector<vector<Point>> &circles, vector<Point> &approx) {
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

    void ShapeFinder::displayShapes(const vector<vector<Point>> &triangles, const vector<vector<Point>> &rectangles,
                                    const vector<vector<Point>> &squares, const vector<vector<Point>> &circles) const {
        drawContours(outputImage, triangles, -1, Scalar(213, 0, 249), 4, 8); //magenta
        drawContours(outputImage, rectangles, -1, Scalar(0, 176, 255), 4, 8); //blue
        drawContours(outputImage, squares, -1, Scalar(255, 255, 0), 4, 8); //yellow
        drawContours(outputImage, circles, -1, Scalar(100, 230, 0), 4, 8); //teal

    }

    void ShapeFinder::displayShapeCountUi(vector<vector<cv::Point>> const &circles,
                                          vector<vector<cv::Point>> const &triangles,
                                          vector<vector<cv::Point>> const &rectangles,
                                          vector<vector<cv::Point>> const &squares) {
        vector<vector<Point>> tp = vector<vector<Point>>({{Point(80, 100), Point(120, 100), Point(100, 60)}});
        circle(outputImage, Point(100, 30), 20, Scalar(0, 0, 255), -1);
        fillPoly(outputImage, tp, Scalar(0, 0, 255));
        Drawing::rectangle(outputImage, Point(80, 130), Point(120, 135), Scalar(0, 0, 255),
                           Scalar(0, 0, 255));
        Drawing::rectangle(outputImage, Point(80, 165), Point(120, 205), Scalar(0, 0, 255),
                           Scalar(0, 0, 255));

        Drawing::text(outputImage, to_string(getCountMode(circleCounts, circles.size(), 30)), Point(20, 50),
                      Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
        Drawing::text(outputImage, to_string(getCountMode(triangleCounts, triangles.size(), 30)),
                      Point(20, 100),
                      Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
        Drawing::text(outputImage, to_string(getCountMode(rectangleCounts, rectangles.size(), 30)),
                      Point(20, 150),
                      Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
        Drawing::text(outputImage, to_string(getCountMode(squareCounts, squares.size(), 30)), Point(20, 200),
                      Scalar(0, 0, 255), Drawing::TOP_LEFT, 2, 4);
    }

    Mat ShapeFinder::preprocessImage(Mat &input) {
        Mat procImg = input;
        //Compute standard deviation for image
        meanStdDev(procImg, mu, sigma);

        //Remove small noise
        Mat element = getStructuringElement(MORPH_RECT,
                                            Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1),
                                            Point(EROSION_SIZE, EROSION_SIZE));

        ImageFilter::downsample(procImg, 2);
        ImageFilter::upsample(procImg, 2);

        //Threshold
        double threshold1 = mu.val[0] - 2.0 * sigma.val[0];
        double threshold2 = mu.val[0] + 0.0 * sigma.val[0];
        Canny(procImg, procImg, threshold1, threshold2);


        dilate(procImg, procImg, element);
        erode(procImg, procImg, element);
        dilate(procImg, procImg, element);

        return procImg;
    }

    ShapeFinder::ShapeFinder(cv::Size
                             frameSize) {
        createTuningWindow();

        // cam.setFrameSize(Size(1280, 720));
        calibrationData = *Camera::loadCalibrationDataFromXML("../config/fisheye_cameracalib.xml",
                                                              std::move(frameSize));
    }

    void ShapeFinder::processFrame(Mat &input) {
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        vector<vector<Point>> triangles;
        vector<vector<Point>> rectangles;
        vector<vector<Point>> squares;
        vector<vector<Point>> circles;

        // update trackbars
        updateTrackbars();

        // undistort
        input = Camera::undistort(input, calibrationData);

        // Convert color for processing
        cvtColor(input, input, COLOR_BGR2GRAY);

        // Threshold for getting black and white values
        Mat thresholdImage;
        threshold(input, thresholdImage, IMAGE_BLACK_THRESHOLD, 255, 0);
        cvtColor(thresholdImage, thresholdImage, cv::COLOR_GRAY2RGB);

        processedImage = preprocessImage(input);

        findContours(processedImage, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_TC89_L1);

        for (auto &contour : contours) {
            vector<Point> approx;
            Contour c = Contour(contour);
            approxPolyDP(contour, approx, EPSILON_APPROX_TOLERANCE_FACTOR * c.arcLength(true), true);

            const Scalar &averageColor = c.averageColor(thresholdImage);
            if (averageColor[0] > CONTOUR_BLACK_THRESHOLD)
                continue;
            if (c.area() > MAX_AREA || c.area() < MIN_AREA)
                continue;

            classifyShape(triangles, rectangles, squares, circles, approx);
        }

        cvtColor(input, outputImage, cv::COLOR_GRAY2RGB);
        displayShapes(triangles, rectangles, squares, circles);
        displayShapeCountUi(circles, triangles, rectangles, squares);

        ImageTransform::scale(outputImage, .5);

        ImageTransform::scale(input, .5);

        imshow("Input", input);
        imshow("Output", outputImage);

        if (waitKey(1) == 32)
            waitKey(0);
    }
}