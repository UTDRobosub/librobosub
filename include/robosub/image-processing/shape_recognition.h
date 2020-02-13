#ifndef LIBROBOSUB_SHAPE_RECOGNITION_H
#define LIBROBOSUB_SHAPE_RECOGNITION_H

namespace robosub {
    class ShapeFindResult {
    private:
        int COUNT_MODE_LOOKBACK = 5;

        static int mode(const deque<int> &q);

    public:
        deque<int> triangleCounts = deque<int>();
        deque<int> squareCounts = deque<int>();
        deque<int> rectangleCounts = deque<int>();
        deque<int> circleCounts = deque<int>();

        vector<vector<Point>> triangles;
        vector<vector<Point>> rectangles;
        vector<vector<Point>> squares;
        vector<vector<Point>> circles;

        static int getCountMode(const deque<int> &previousCounts);

        void addCount(deque<int> &previousCounts, int newCount);

        int getLastTriangleCount();

        int getLastSquareCount();

        int getLastRectangleCount();

        int getLastCircleCount();
    };


    class ShapeFinder {
    private:
        bool running = true;

        Camera::CalibrationData calibrationData;

        Mat outputImage, processedImage;
        Scalar mu, sigma;

        void classifyShape(ShapeFindResult &result, vector<Point> &approx);

        Mat preprocessImage(Mat &input);

    public:
        double EPSILON_APPROX_TOLERANCE_FACTOR = 0.0425;
        double MIN_AREA = 50;
        double MAX_AREA = 8220;
        double SQUARE_RATIO_THRESHOLD = .72;
        double TRIANGLE_RATIO_THRESHOLD = .22;
        int EROSION_SIZE = 1;

        // TODO: Tune these parameters. Current values are okayish
        double IMAGE_BLACK_THRESHOLD = 38;
        double CONTOUR_BLACK_THRESHOLD = 150;

        explicit ShapeFinder(Camera::CalibrationData calibrationData);

        void processFrame(Mat &input, ShapeFindResult &result);
    };

}

#endif //LIBROBOSUB_SHAPE_RECOGNITION_H
