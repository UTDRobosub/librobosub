#include <opencv2/opencv.hpp>
#include <opencv2/ccalib/omnidir.hpp> //omni (very high FOV) calibration
#include <robosub/robosub.h>

using namespace std;
using namespace cv;
using namespace robosub;

enum Model {
    PINHOLE,
    FISHEYE,
    OMNI
};



bool detectAndParseChessboard(const cv::Mat &grey, cv::Mat &frame, Size &patternSize, vector<Point2f> &currentImagePoints)
{
    int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK;
    bool isTemplateFound = cv::findChessboardCorners(grey, patternSize, currentImagePoints, chessBoardFlags);

    if (isTemplateFound) {
        cv::cornerSubPix(grey, currentImagePoints, cv::Size(11,11),
            cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
        cv::drawChessboardCorners(frame, patternSize, cv::Mat(currentImagePoints), isTemplateFound);
        // mTemplateLocations.insert(mTemplateLocations.begin(), currentImagePoints[0]);
    }
    return isTemplateFound;
}

cv::Ptr<cv::aruco::Dictionary> mArucoDictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME(capParams.charucoDictName));
cv::Ptr<cv::aruco::CharucoBoard> mCharucoBoard = cv::aruco::CharucoBoard::create(mBoardSize.width, mBoardSize.height, capParams.charucoSquareLenght, capParams.charucoMarkerSize, mArucoDictionary);

bool detectAndParseChAruco(const cv::Mat &frame, vector<Point2f> &currentImagePoints, vector<Point3f> &currentObjectPoints)
{
#ifdef HAVE_OPENCV_ARUCO
    cv::Ptr<cv::aruco::Board> board = mCharucoBoard.staticCast<cv::aruco::Board>();
    vector<Mat> currentCharucoCorners, currentCharucoIds;

    std::vector<std::vector<cv::Point2f> > corners, rejected;
    std::vector<int> ids;
    cv::aruco::detectMarkers(frame, mArucoDictionary, corners, ids, cv::aruco::DetectorParameters::create(), rejected);
    cv::aruco::refineDetectedMarkers(frame, board, corners, ids, rejected);
    cv::Mat currentCharucoCorners, currentCharucoIds;
    if(ids.size() > 0)
        cv::aruco::interpolateCornersCharuco(corners, ids, frame, mCharucoBoard, currentCharucoCorners,
                                         currentCharucoIds);
    if(ids.size() > 0) cv::aruco::drawDetectedMarkers(frame, corners);

    //get object and image points
    cv::aruco::getBoardObjectAndImagePoints(board, currentCharucoCorners, currentCharucoIds, currentObjectPoints, currentImagePoints);

    if(currentCharucoCorners.total() > 3) {
        float centerX = 0, centerY = 0;
        for (int i = 0; i < currentCharucoCorners.size[0]; i++) {
            centerX += currentCharucoCorners.at<float>(i, 0);
            centerY += currentCharucoCorners.at<float>(i, 1);
        }
        centerX /= currentCharucoCorners.size[0];
        centerY /= currentCharucoCorners.size[0];

        // mTemplateLocations.insert(mTemplateLocations.begin(), cv::Point2f(centerX, centerY));
        cv::aruco::drawDetectedCornersCharuco(frame, currentCharucoCorners, currentCharucoIds);

        return true;
    }
#else
    (void)frame;
#endif
    return false;
}

int main(int argc, char** argv) {
    //parse command line arguments
    const String keys =
        "{help ?         |         | print this message }"
        "{vc cols        |1920     | image columns }"
        "{vr rows        |1080     | image rows }"
        "{c cam camera   |0        | camera id }"
        "{m model        |omni     | use calibration model: 'pinhole', 'fisheye', 'omni' (default) }"
        "{flip           |false    | flip input frames vertically }"
        "{scale          |1        | visualization scale }"
    ;
    CommandLineParser parser(argc, argv, keys);
    parser.about("Camera Calibration");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    if (!parser.check())
  	{
    		parser.printErrors();
    		return 0;
  	}

    String m = parser.get<String>("m");
    Model model;
    if (m == "pinhole") model = Model::PINHOLE;
    else if (m == "fisheye") model = Model::FISHEYE;
    else if (m == "omni") model = Model::OMNI;
    else {
        parser.printMessage();
        return 0;
    }
    int cameraId = parser.get<int>("c");
    bool flip = parser.get<bool>("flip");
    double scale = parser.get<double>("scale");
    Camera* cam = new Camera(cameraId);
    Size output = cam->setFrameSizeToMaximum();
    cout << output << endl;

    Mat frame, grey, undistorted;
    int key;

    //prepare parameters
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;
    Mat K, D, xi, idx;
    TermCriteria criteria(TermCriteria::COUNT + TermCriteria::EPS, 200, 0.0001);
    vector<Mat> rvecs, tvecs;
    Size patternSize(8,6); //number of chessboard squares on board
    float squareSize = 16.3; //mm

    cout << "Press [SPACE] to take image" << endl;

    while (true)
    {
      cam->retrieveFrameBGR(frame);
      frame = frame.clone();

      if (flip) ImageTransform::flip(frame, ImageTransform::FlipAxis::HORIZONTAL);
      if (scale != 1) ImageTransform::scale(frame, scale);

      imshow("Uncalibrated Image", frame);

      //convert to grayscale
      cvtColor(frame, grey, COLOR_BGR2GRAY, CV_8UC1);

      //search for chessboard pattern
      vector<Point2f> currentImagePoints;
      vector<Point3f> currentObjectPoints;
      bool patternFound = detectAndParseChAruco(grey, frame, patternSize, currentImagePoints, currentObjectPoints);
      if(!patternFound) {
        cout << "No pattern found" << endl;
      }

      //wait for spacebar to take photo
      key = waitKey(1);
      if (patternFound && key == 32) {
        //take image
        cout << "Take image" << endl;

        //add object points
        imagePoints.push_back(currentImagePoints);
        objectPoints.push_back(currentObjectPoints);

        if (model == Model::OMNI) {
          //calibrate
          double rms = cv::omnidir::calibrate(objectPoints, imagePoints, frame.size(), K, xi, D, rvecs, tvecs, 0, criteria, idx);
          cout << "RMS: " << rms << endl;
          //undistort image
          Size newSize;
          Mat Knew;
          cv::omnidir::undistortImage(frame, undistorted, K, D, xi, cv::omnidir::RECTIFY_STEREOGRAPHIC, Knew, newSize);
          cout << newSize << endl;
          imshow("Calibrated Image", undistorted);
        }

        //fisheye::calibrate(objectPoints, imagePoints, image.size(), cameraMatrix, distortionMatrix, rvec, tvec, flag); // Calibration
        //fisheye::undistortImage(image, dstImage, cameraMatrix, distortionMatrix);
      }


    }
}
