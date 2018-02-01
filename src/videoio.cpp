#include "robosub/videoio.h"

namespace robosub {
	void Camera::updateRetrieveTime()
	{
		if (startTime == 0) startTime = Time::millis();
		frame++;
		lastTime = Time::millis();
		fps->frame();
	}

	bool Camera::testLiveStream() {
		if (!isOpen()) return false;
		liveStream = cap->get(cv::CAP_PROP_POS_FRAMES) < 0 || cap->get(cv::CAP_PROP_FPS) == 0;
		init = true;
		return liveStream;
	}

	Camera::Camera(int index)
	{
		cap = new VideoCapture(index);
		fps = new FPS();
		frame = 0;
		startTime = 0;
		lastTime = Time::millis();
		testLiveStream();
	}

	Camera::Camera(string file)
	{
		cap = new VideoCapture(file);
		fps = new FPS();
		frame = 0;
		startTime = 0;
		lastTime = Time::millis();
		testLiveStream();
	}

	Camera::~Camera()
	{
		delete cap;
		delete fps;
	}

	bool Camera::isOpen()
	{
		return cap->isOpened();
	}

	bool Camera::grabFrame()
	{
		return cap->grab();
	}

	bool Camera::retrieveFrameBGR(Mat& img)
	{
	    #ifdef WINDOWS
            if (!cap->retrieve(img)) return false;
        #else
            if (!cap->read(img)) return false;
        #endif
		updateRetrieveTime();
		return true;
	}

	bool Camera::retrieveFrameGrey(Mat& img)
	{
        #ifdef WINDOWS
            if (!cap->retrieve(img)) return false;
        #else
            if (!cap->read(img)) return false;
        #endif
		updateRetrieveTime();
		cvtColor(img, img, COLOR_BGR2GRAY, CV_8UC1);
		return true;
	}

	Camera::CalibrationData::CalibrationData()
	{
		this->cameraResolution = Size();
		this->distortionMatrix = Mat();
		this->cameraMatrix = Mat();
	}
	Camera::CalibrationData::CalibrationData(Size cameraResolution)
	{
		this->cameraMatrix = Mat::eye(3, 3, CV_64F);
		cameraMatrix.at<double>(0, 0) = 0.5;
		cameraMatrix.at<double>(1, 1) = 0.5;
		cameraMatrix.at<double>(0, 2) = (double)cameraResolution.width / 2.0;
		cameraMatrix.at<double>(1, 2) = (double)cameraResolution.height / 2.0;
		this->distortionMatrix = Mat::zeros(5, 1, CV_64F);
		this->cameraResolution = cameraResolution;
	}

	Camera::CalibrationData* Camera::loadCalibrationDataFromXML(string filename)
	{
		FileStorage fs;
		CalibrationData* calibData = new CalibrationData();
		if (!Util::fileExists(filename)) throw std::invalid_argument("File does not exist");
		fs.open(filename, FileStorage::READ);
		fs["cameraMatrix"] >> calibData->cameraMatrix;
		fs["dist_coeffs"] >> calibData->distortionMatrix;
		fs["cameraResolution"] >> calibData->cameraResolution;
		fs.release();
		assert(calibData->cameraMatrix.cols == 3 && calibData->cameraMatrix.rows == 3);
		assert(calibData->distortionMatrix.cols == 5 && calibData->distortionMatrix.rows == 1);
		//verify aspect ratio
		assert((calibData->cameraResolution.width / calibData->cameraResolution.height) ==
			(this->getFrameSize().width / this->getFrameSize().height));
		//scale camera params by change in frame size
		//calibData->cameraMatrix *= (double)this->getFrameSize().width / (double)calibData->cameraResolution.width;
		return calibData;
	}

	Mat Camera::undistort(Mat& frame, CalibrationData& calib)
	{
		//THE PROBLEM: Selection is actually done on an UNDISTORTED image but expects results in the distored plane!
		Mat output;
		cv::undistort(frame, output, calib.cameraMatrix, calib.distortionMatrix);
		return output;
	}

	void Camera::undistortPoints(InputArray & points, OutputArray & pointsOptimal, CalibrationData & calib)
	{
		cv::undistortPoints(points, pointsOptimal, calib.cameraMatrix, calib.distortionMatrix);
	}

	double Camera::getFrameRate()
	{
		if (isLiveStream()) return fps->fps();
		return cap->get(cv::CAP_PROP_FPS);
	}

	cv::Size Camera::getFrameSize() {
		if (!isOpen()) return cv::Size(0, 0);
		return cv::Size(cap->get(cv::CAP_PROP_FRAME_WIDTH), cap->get(cv::CAP_PROP_FRAME_HEIGHT));
	}

	cv::Size Camera::setFrameSize(cv::Size size)
	{
		if (isOpen()) {
			cap->set(cv::CAP_PROP_FRAME_WIDTH, size.width);
			cap->set(cv::CAP_PROP_FRAME_HEIGHT, size.height);
		}
		return getFrameSize();
	}

	cv::Size Camera::setFrameSizeToMaximum()
	{
		return setFrameSize(cv::Size(INT_MAX, INT_MAX));
	}

	long Camera::getPositionFrame()
	{
		if (isLiveStream()) return frame;
		return cap->get(cv::CAP_PROP_POS_FRAMES);
	}

	double Camera::getPositionSeconds()
	{
		if (isLiveStream()) return (lastTime - startTime) / 1000.0;
		return cap->get(cv::CAP_PROP_POS_MSEC) / 1000.0;
	}

	long Camera::getFrameCount()
	{
		if (isLiveStream()) return 0;
		return cap->get(cv::CAP_PROP_FRAME_COUNT);
	}

	bool Camera::isLiveStream()
	{
		if (!init) return testLiveStream();
		return liveStream;
	}
}
