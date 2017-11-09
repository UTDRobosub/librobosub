#pragma once

#include "common.h"
#include "time.h"
#include "fps.h"
#include <opencv2/opencv.hpp>

namespace robosub
{
	class Camera
	{
	private:
		int frame;
		long long startTime;
		long long lastTime;
		bool liveStream = false;
		bool init = false;

		void updateRetrieveTime();
		bool testLiveStream();

		VideoCapture* cap;
		FPS* fps;

	public:
		///Begin capturing using a live camera feed
		///Index 0 is the primary camera, 1 is secondary, etc.
		EXPORT Camera(int index);
		///Begin capturing using a recorded file or IP camera
		EXPORT Camera(cv::String file);
		///Camera destructor
		EXPORT ~Camera();

		///Check if the camera
		EXPORT bool isOpen();
		///Grab and store a frame without decoding it.
		///When using multiple cameras, call this on all cameras before retrieving frames.
		EXPORT bool grabFrame();
		///Retrieve and decode the current frame (RGBA)
		EXPORT bool retrieveFrameBGR(Mat& img);
		///Retrieve and decode the current frame (greyscale)
		EXPORT bool retrieveFrameGrey(Mat& img);

		///Get stream frame rate
		EXPORT double getFrameRate();
		//Get frame size
		EXPORT cv::Size getFrameSize();
		///Attempt to set frame size. Returns actual size set.
		EXPORT cv::Size setFrameSize(cv::Size size);
		///Set frame size to largest possible
		EXPORT cv::Size setFrameSizeToMaximum();
		///Get frame position
		///If live stream, will return number of frames retrieved
		EXPORT long getPositionFrame();
		///Get position in seconds
		///If live stream, will return timestamp of last frame
		EXPORT double getPositionSeconds();
		///Get number of frames
		EXPORT long getFrameCount();
		///Returns true if live stream, false if reading from file
		EXPORT bool isLiveStream();
	};
}
