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

	Camera::Camera(cv::String file)
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
