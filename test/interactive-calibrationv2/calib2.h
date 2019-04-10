#pragma once

#include <opencv2/opencv.hpp>

#ifndef HAVE_OPENCV_ARUCO
#error OpenCV aruco module is required.
#endif
#ifndef HAVE_OPENCV_CCALIB
#error OpenCV ccalib module is required.
#endif

#include <opencv2/aruco/charuco.hpp>
#include <opencv2/ccalib/omnidir.hpp> //omni (very high FOV) calibration
#include <robosub/robosub.h>

using namespace std;
using namespace cv;
using namespace robosub;

cv::Ptr<cv::aruco::Dictionary> mArucoDictionary;
cv::Ptr<cv::aruco::CharucoBoard> mCharucoBoard;

const Size boardSize = Size(9, 6);
const double squareSize = 23.0; //mm

enum Model {
    PINHOLE,
    FISHEYE,
    OMNI
};

enum BoardType {
    CHECKERBOARD,
    CHARUCO
};

struct CalibrationData
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat stdDeviations;
    cv::Mat perViewErrors;
    cv::Mat omniXi;
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    double totalAvgErr;
    Size frameSize;

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    std::vector<cv::Mat> allCharucoCorners;
    std::vector<cv::Mat> allCharucoIds;

    cv::Mat undistMap1, undistMap2;

    CalibrationData() { }
};

struct CameraParameters
{
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    double avgError;

    CameraParameters(){}
    CameraParameters(cv::Mat& _cameraMatrix, cv::Mat& _distCoeffs, double _avgError = 0) :
            cameraMatrix(_cameraMatrix), distCoeffs(_distCoeffs), avgError(_avgError)
    {}
};

struct CaptureParameters
{
    Model calibModel;
    BoardType boardType;
    cv::Size boardSize;
    int charucoDictName;
    float charucoSquareLength, charucoMarkerSize;
    string outputFilename;

    CaptureParameters() { }
};