//
// Created by ryan on 10/9/19.
//

#ifndef LIBROBOSUB_SHAPE_RECOGNITION_H
#define LIBROBOSUB_SHAPE_RECOGNITION_H

namespace robosub {
    class ShapeFinder {
    private:
        bool running = true;

        double EPSILON_APPROX_TOLERANCE_FACTOR = 0.0425;
        double MIN_AREA = 50;
        double MAX_AREA = 8220;
        double SQUARE_RATIO_THRESHOLD = .72;
        double TRIANGLE_RATIO_THRESHOLD = .22;
        int EROSION_SIZE = 1;

        // TODO: Tune these parameters. Current values are okayish
        double IMAGE_BLACK_THRESHOLD = 38;
        double CONTOUR_BLACK_THRESHOLD = 150;

        Camera::CalibrationData calibrationData;
        deque<int> triangleCounts = deque<int>();
        deque<int> squareCounts = deque<int>();
        deque<int> rectangleCounts = deque<int>();
        deque<int> circleCounts = deque<int>();

        Mat outputImage, processedImage;
        Scalar mu, sigma;

        void createTuningWindow();

        void updateTrackbars();

        void classifyShape(vector<vector<Point>> &triangles, vector<vector<Point>> &rectangles,
                           vector<vector<Point>> &squares,
                           vector<vector<Point>> &circles, vector<Point> &approx);

        void displayShapes(const vector<vector<Point>> &triangles, const vector<vector<Point>> &rectangles,
                           const vector<vector<Point>> &squares, const vector<vector<Point>> &circles) const;

        void displayShapeCountUi(const vector<vector<cv::Point>> &circles, const vector<vector<cv::Point>> &triangles,
                                 const vector<vector<cv::Point>> &rectangles, const vector<vector<cv::Point>> &squares);

        Mat preprocessImage(Mat &input);

    public:
        explicit ShapeFinder(cv::Size);

        void processFrame(Mat &input);
    };
}

#endif //LIBROBOSUB_SHAPE_RECOGNITION_H
