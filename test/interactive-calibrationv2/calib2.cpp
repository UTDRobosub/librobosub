#include "calib2.h"

static bool loadCaptureParams(CommandLineParser& parser, CaptureParameters& capParams) {
    //model type
    String m = parser.get<String>("m");
    Model model;
    if (m == "pinhole") model = Model::PINHOLE;
    else if (m == "fisheye") model = Model::FISHEYE;
    else if (m == "omni") {
        model = Model::OMNI;
        cout << "Omni currently not supported" << endl;
        return false;
    }
    else return false;

    capParams.outputFilename = parser.get<string>("f");

    capParams.calibModel = model;
    capParams.boardSize = cv::Size(8, 6);
    capParams.charucoDictName = 0;
    capParams.charucoSquareLength = 200;
    capParams.charucoMarkerSize = 100;

    return true;
}

void prepareChArucoDictionary(const CaptureParameters& capParams) {
    mArucoDictionary = cv::aruco::getPredefinedDictionary(
            cv::aruco::PREDEFINED_DICTIONARY_NAME(capParams.charucoDictName));
    mCharucoBoard = cv::aruco::CharucoBoard::create(capParams.boardSize.width, capParams.boardSize.height,
                                                    capParams.charucoSquareLength, capParams.charucoMarkerSize, mArucoDictionary);
}

bool detectAndParseChAruco(const cv::Mat &frame, std::vector<cv::Point3f> &currentObjectPoints, std::vector<cv::Point2f> &currentImagePoints,
                           cv::Mat &currentCharucoCorners, cv::Mat &currentCharucoIds)
{
    cv::Ptr<cv::aruco::Board> board = mCharucoBoard.staticCast<cv::aruco::Board>();

    std::vector<std::vector<cv::Point2f>> corners, rejected;
    std::vector<int> ids;
    cv::aruco::detectMarkers(frame, mArucoDictionary, corners, ids, cv::aruco::DetectorParameters::create(), rejected);
    cv::aruco::refineDetectedMarkers(frame, board, corners, ids, rejected);
    if(ids.size() > 0) {
        cv::aruco::interpolateCornersCharuco(corners, ids, frame, mCharucoBoard, currentCharucoCorners,
                                             currentCharucoIds);
        cv::aruco::drawDetectedMarkers(frame, corners);
        cv::aruco::getBoardObjectAndImagePoints(board, currentCharucoCorners, currentCharucoIds,
                                                currentObjectPoints, currentImagePoints);
    }

    if(currentCharucoCorners.total() > 3) {
        cv::aruco::drawDetectedCornersCharuco(frame, currentCharucoCorners, currentCharucoIds);
        return true;
    }

    return false;
}

template<typename _Tp> static cv::Mat toMat(const vector<vector<_Tp> > vecIn, const int type) {
    cv::Mat mat(vecIn.size(), vecIn.at(0).size(), type);
    for(int i=0; i<mat.rows; ++i)
        for(int j=0; j<mat.cols; ++j)
            mat.at<_Tp>(i, j) = vecIn.at(i).at(j);
    return mat;
}

void recalibrate(const Ptr<CalibrationData>& calibData, Model model) {
    TermCriteria termCriteria = cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 100, 1e-4);

    cv::aruco::calibrateCameraCharuco(calibData->allCharucoCorners, calibData->allCharucoIds,
                                      mCharucoBoard, calibData->frameSize,
                                      calibData->cameraMatrix, calibData->distCoeffs,
                                      cv::noArray(), cv::noArray(), cv::noArray(), cv::noArray(),
                                      cv::noArray(), 0, termCriteria);

    switch(model) {
        case PINHOLE:
            cv::Mat P = cv::getOptimalNewCameraMatrix(calibData->cameraMatrix, calibData->distCoeffs,
                                                      calibData->frameSize, 0.0, calibData->frameSize);
            cv::initUndistortRectifyMap(calibData->cameraMatrix, calibData->distCoeffs, cv::noArray(), P,
                                        calibData->frameSize, CV_16SC2, calibData->undistMap1, calibData->undistMap2);
//        case FISHEYE:

    }

//    if (mCalibType == Pinhole) {
//
//    } else if (mCalibType == Fisheye) {
//        cv::Mat P;
//        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(mCalibData->cameraMatrix, mCalibData->distCoeffs,
//                                                                mCalibData->imageSize,cv::noArray(), P);
//        cv::fisheye::initUndistortRectifyMap(mCalibData->cameraMatrix, mCalibData->distCoeffs, cv::noArray(),
//                                             P, mCalibData->imageSize, CV_16SC2, mCalibData->undistMap1, mCalibData->undistMap2);
//    } else if (mCalibType == Omni) {
//        cv::omnidir::initUndistortRectifyMap(mCalibData->cameraMatrix, mCalibData->distCoeffs, mCalibData->omniXi,
//                                             cv::noArray(), cv::getOptimalNewCameraMatrix(mCalibData->cameraMatrix, mCalibData->distCoeffs, mCalibData->imageSize, 0.0, mCalibData->imageSize), mCalibData->imageSize, CV_16SC2, mCalibData->undistMap1, mCalibData->undistMap2,
//                                             cv::omnidir::RECTIFY_STEREOGRAPHIC);
//    }

//    switch(model) {
//        case PINHOLE:
//            calibData->totalAvgErr = cv::calibrateCamera(calibData->objectPoints, calibData->imagePoints, calibData->frameSize,
//                                                         calibData->cameraMatrix, calibData->distCoeffs, rvec, tvec, 0, termCriteria);
//            break;
//        case FISHEYE:
//            calibData->totalAvgErr = cv::fisheye::calibrate(calibData->objectPoints, calibData->imagePoints, calibData->frameSize,
//                                                            calibData->cameraMatrix, calibData->distCoeffs, rvec, tvec, cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC, termCriteria);
//            break;
//        case OMNI:
//            calibData->totalAvgErr = cv::omnidir::calibrate(toMat<Point3f>(calibData->objectPoints, CV_32FC3), toMat<Point2f>(calibData->imagePoints, CV_32FC2),
//                                                            calibData->frameSize,
//                                                            calibData->cameraMatrix, calibData->omniXi, calibData->distCoeffs, rvec, tvec,
//                                                            0, termCriteria);
//            break;
//    }


    cout << calibData->cameraMatrix << endl;
    cout << calibData->distCoeffs << endl;

    cout << calibData->objectPoints.size() << " images used in calibration" << endl;
}

bool saveCameraParametersToFile(const Ptr<CalibrationData>& calibData, CaptureParameters& capParams)
{
    bool success = false;
    if(calibData->cameraMatrix.total()) {
        cv::FileStorage parametersWriter(capParams.outputFilename, cv::FileStorage::WRITE);
        if(parametersWriter.isOpened()) {
            time_t rawtime;
            time(&rawtime);
            char buf[256];
            strftime(buf, sizeof(buf)-1, "%c", localtime(&rawtime));

            string model;
            switch(capParams.calibModel) {
                case PINHOLE: model = "pinhole"; break;
                case FISHEYE: model = "fisheye"; break;
                case OMNI: model = "omni"; break;
            }

            parametersWriter << "calibrationDate" << buf;
            parametersWriter << "calibrationModel" << model;
            parametersWriter << "framesCount" << (int)calibData->objectPoints.size();
            parametersWriter << "cameraResolution" << calibData->frameSize;
            parametersWriter << "cameraMatrix" << calibData->cameraMatrix;
            parametersWriter << "distortionMatrix" << calibData->distCoeffs;
            parametersWriter << "avgReprojectionError" << calibData->totalAvgErr;

            parametersWriter.release();
            success = true;
        }
    }
    return success;
}

int main(int argc, char** argv) {
    //parse command line arguments
    const String keys =
            "{help ?         |                   | print this message }"
            "{vc cols        |1280               | image columns }"
            "{vr rows        |720                | image rows }"
            "{c cam camera   |0                  | camera id }"
            "{m model        |pinhole            | use calibration model: 'pinhole' (default), 'fisheye', 'omni' }"
            "{f filename     |cameraParams.xml   | save camera parameters to file }"
            "{flip           |false              | flip input frames vertically }";
    CommandLineParser parser(argc, argv, keys);
    parser.about("Camera Calibration");
    if (parser.has("help")) {
        parser.printMessage();
        return 0;
    }
    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    //load system parameters
    CaptureParameters capParams;
    bool ok = loadCaptureParams(parser, capParams);
    if (!ok) {
        parser.printMessage();
        return 0;
    }

    int cameraId = parser.get<int>("c");
    bool flip = parser.get<bool>("flip");
    Camera *cam = new Camera(cameraId);
    Size frameSize = Size(parser.get<int>("vc"), parser.get<int>("vr"));
    frameSize = cam->setFrameSize(frameSize);
    cout << frameSize << endl;

    //prepare dictionary
    prepareChArucoDictionary(capParams);

    cout << "Press [SPACE] to take image" << endl;
    cout << "Press [s]     to save calibration data" << endl;
    cout << "Press [z]     to undo last image" << endl;
    cout << endl;

    //prepare state variables
    Mat frame, gray, undistorted;
    Ptr<CalibrationData> calibData(new CalibrationData);
    calibData->frameSize = frameSize;
    char key;

    while (true) {
        //retrieve frame
        cam->retrieveFrameBGR(frame);
        frame = frame.clone();

        //convert to grayscale
        Camera::convertFrameToGrayscale(frame, gray);

        //search for charuco board
        vector<Point3f> currentObjectPoints;
        vector<Point2f> currentImagePoints;
        cv::Mat currentCharucoCorners, currentCharucoIds;
        bool boardDetected = detectAndParseChAruco(frame, currentObjectPoints, currentImagePoints,
                                                   currentCharucoCorners, currentCharucoIds);

        //display uncalibrated image
        if (flip) ImageTransform::flip(frame, ImageTransform::FlipAxis::HORIZONTAL);
        imshow("Uncalibrated Image", frame);

        //perform operation on keypress
        key = (char)cv::waitKey(1);

        //take frame
        if (key == ' ' && boardDetected) {
            cout << "Image taken" << endl;

            imshow("Last Image Taken", frame);

            calibData->objectPoints.push_back(currentObjectPoints);
            calibData->imagePoints.push_back(currentImagePoints);
            calibData->allCharucoCorners.push_back(currentCharucoCorners);
            calibData->allCharucoIds.push_back(currentCharucoIds);
            recalibrate(calibData, capParams.calibModel);

        //remove frame
        } else if ((key == 'z') && (!calibData->objectPoints.empty())) {
            calibData->objectPoints.pop_back();
            calibData->imagePoints.pop_back();
            calibData->allCharucoCorners.pop_back();
            calibData->allCharucoIds.pop_back();
            recalibrate(calibData, capParams.calibModel);
        //save data
        } else if ((key == 's') && (!calibData->objectPoints.empty())) {
            if (!saveCameraParametersToFile(calibData, capParams)) {
                cout << "Failed to save camera parameters" << endl;
            } else {
                cout << "Saved camera parameters!" << endl;
            }
        }

        //remap
        if (!calibData->objectPoints.empty()) {

            switch(capParams.calibModel) {
                case PINHOLE:
                    cv::remap(frame, undistorted, calibData->undistMap1, calibData->undistMap2, cv::INTER_LINEAR);
                    break;
//                case PINHOLE:
//                    cv::undistort(frame, undistorted, calibData->cameraMatrix, calibData->distCoeffs, calibData->cameraMatrix);
//                    break;
//                case FISHEYE:
//                    cv::fisheye::undistortImage(frame, undistorted, calibData->cameraMatrix, calibData->distCoeffs, calibData->cameraMatrix);
//                    break;
//                case OMNI:
//                    cv::omnidir::undistortImage(frame, undistorted, calibData->cameraMatrix, calibData-> distCoeffs, calibData->omniXi,
//                                                cv::omnidir::RECTIFY_PERSPECTIVE, calibData->cameraMatrix);
//                    break;
            }

            imshow("Calibrated Image", undistorted);
        }
    }
}