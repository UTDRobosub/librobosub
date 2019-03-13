#pragma once

#include "common.h"
#include "time.h"
#include "fps.h"
#include "util.h"
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

		///Camera calibration information
		struct CalibrationData {
			enum Model {
				PINHOLE,
				FISHEYE
			};

			Mat cameraMatrix;
			Mat distortionMatrix;
			Size cameraResolution;
			Model model;

			EXPORT CalibrationData();
			EXPORT CalibrationData(Size cameraResolution, Model model);
			EXPORT CalibrationData(Mat cameraMatrix, Mat distortionMatrix, Size cameraResolution, Model model) {
				this->cameraMatrix = cameraMatrix;
				this->distortionMatrix = distortionMatrix;
				this->cameraResolution = cameraResolution;
				this->model = model;
			}
		};

		///Begin capturing using a live camera feed
		///Index 0 is the primary camera, 1 is secondary, etc.
		EXPORT Camera(int index);
		///Begin capturing using a recorded file or IP camera
		EXPORT Camera(string file);
		///Camera destructor
		EXPORT ~Camera();

		///Check if the camera is open
		EXPORT bool isOpen();
		///Grab and store a frame without decoding it.
		///When using multiple cameras, call this on all cameras before retrieving frames.
		EXPORT bool grabFrame();
		///Retrieve a grabbed frame
		EXPORT bool getGrabbedFrame(Mat& img);
		///Retrieve and decode the current frame (RGBA)
		EXPORT bool retrieveFrameBGR(Mat& img);
		///Retrieve and decode the current frame (greyscale)
		EXPORT bool retrieveFrameGrey(Mat& img);
        ///Convert a frame from BGR to Grayscale
        EXPORT static void convertFrameToGrayscale(Mat& bgr, Mat& gray);

		///Prepare single-camera calibration data from XML file
		///This method will also scale camera parameters appropriately for the camera
		EXPORT static CalibrationData* loadCalibrationDataFromXML(const string filename, const Size frameSize);
		///Undistort frame
		EXPORT static Mat undistort(Mat& input, CalibrationData& calib);
		///Compute optimal undistorted points
		EXPORT static void undistortPoints(InputArray& points, OutputArray& undistortedPoints, CalibrationData& calib);

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
