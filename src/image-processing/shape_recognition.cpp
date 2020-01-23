//
// Created by ryan on 10/9/19.
//

#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <utility>
#include <robosub/image-processing/shape_recognition.h>


using namespace std;

namespace robosub {
    int ShapeFindResult::mode(deque<int> const &q) {
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

    int ShapeFindResult::getCountMode(const deque<int> &previousCounts) {
        if (previousCounts.empty())
            return -1;
        else
            return mode(previousCounts);
    }

    void ShapeFindResult::addCount(deque<int> &previousCounts, int newCount) {
        previousCounts.push_back(newCount);
        while (previousCounts.size() > this->COUNT_MODE_LOOKBACK) {
            previousCounts.pop_front();
        }
    }

    int ShapeFindResult::getLastTriangleCount() {
        if (triangleCounts.size() == 0) return 0;

        return triangleCounts.at(triangleCounts.size() - 1);
    }

    int ShapeFindResult::getLastSquareCount() {
        if (squareCounts.size() == 0) return 0;

        return squareCounts.at(squareCounts.size() - 1);
    }

    int ShapeFindResult::getLastRectangleCount() {
        if (rectangleCounts.size() == 0) return 0;

        return rectangleCounts.at(rectangleCounts.size() - 1);
    }

    int ShapeFindResult::getLastCircleCount() {
        if (circleCounts.size() == 0) return 0;

        return circleCounts.at(circleCounts.size() - 1);
    }

    void ShapeFinder::classifyShape(ShapeFindResult &result, vector<Point> &approx) {
        if (approx.size() >= 5) {
            result.circles.push_back(approx);
        } else if (approx.size() == 4) {
            Rectangle r = Rectangle(approx);
            double dimensionRatio = r.height() / r.width();
            if (dimensionRatio > 1) dimensionRatio = 1 / dimensionRatio;

            if (dimensionRatio < SQUARE_RATIO_THRESHOLD)
                result.rectangles.push_back(approx);
            else
                result.squares.push_back(approx);
        } else if (approx.size() == 3) {
            Triangle t = Triangle(approx);
            double dimensionRatio = t.height() / t.width();

            if (dimensionRatio < TRIANGLE_RATIO_THRESHOLD)
                result.rectangles.push_back(approx);
            else
                result.triangles.push_back(approx);
        } else {
            result.rectangles.push_back(approx);
        }
    }


    Mat ShapeFinder::preprocessImage(Mat &input) {
        Mat procImg = input;
        // Compute standard deviation for image
        meanStdDev(procImg, mu, sigma);

        // Remove small noise
        ImageFilter::downsample(procImg, 2);
        ImageFilter::upsample(procImg, 2);

        // Threshold
        double threshold1 = mu.val[0] - 2.0 * sigma.val[0];
        double threshold2 = mu.val[0] + 0.0 * sigma.val[0];
        Canny(procImg, procImg, threshold1, threshold2);

        // Erosion and dilation
        Mat element = getStructuringElement(MORPH_RECT,
                                            Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1),
                                            Point(EROSION_SIZE, EROSION_SIZE));

        dilate(procImg, procImg, element);
        erode(procImg, procImg, element);
        dilate(procImg, procImg, element);

        return procImg;
    }

    ShapeFinder::ShapeFinder(Camera::CalibrationData calibrationData) {
        this->calibrationData = std::move(calibrationData);
    }

    void ShapeFinder::processFrame(Mat &input, ShapeFindResult &result) {
        result.triangles.clear();
        result.rectangles.clear();
        result.squares.clear();
        result.circles.clear();
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

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

            classifyShape(result, approx);
        }

        result.addCount(result.triangleCounts, result.triangles.size());
        result.addCount(result.squareCounts, result.squares.size());
        result.addCount(result.rectangleCounts, result.rectangles.size());
        result.addCount(result.circleCounts, result.circles.size());
    }
}