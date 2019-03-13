#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <signal.h>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace robosub;

bool running = true;

void catchSignal(int signal) {
	running = false;
}

int main(int argc, char** argv)
{
	//catch signal
	signal(SIGINT, catchSignal);

	Camera cam0 = Camera(1 + CAP_ANY);
	Camera cam1 = Camera(0 + CAP_ANY);

	if (!cam0.isOpen()){
        cout<<"Camera 0 failed to open"<<endl;
        return -1;
	}
	if (!cam1.isOpen()){
        cout<<"Camera 1 failed to open"<<endl;
        return -1;
	}

	cout << cam0.setFrameSize(cv::Size(320, 240)) << endl;
	cam1.setFrameSize(cam0.getFrameSize());

	//calibration data for both cameras is currently the same
	Camera::CalibrationData calibrationData = *Camera::loadCalibrationDataFromXML("../config/stereo-right-640px.xml", cam0.getFrameSize());
	Mat _frame0, _frame1, left, right, left_disp, right_disp, filtered_disp;

	left.create(cam0.getFrameSize(), CV_8U);
	right.create(cam1.getFrameSize(), CV_8U);

	int max_disp = 160; //maximum disparity
	max_disp /= 2;
	if (max_disp % 16 != 0)
		max_disp += 16 - (max_disp % 16);
	double vis_mult = 12.0; //output disparity scale factor
	double wls_lambda = 8000.0;
	double wls_sigma = 1.5;
	int wsize = 5;

	//Create confidence map
	Mat conf_map = Mat(left.rows, left.cols, CV_8U);
	conf_map = Scalar(255);

	Rect ROI;

	//create matchers
	Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, max_disp, wsize);
	left_matcher->setBlockSize(wsize);
	left_matcher->setP1(24 * wsize*wsize);
	left_matcher->setP2(96 * wsize*wsize);
	left_matcher->setPreFilterCap(12);
	left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
	Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);

	//create disparity filter
	Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
	wls_filter->setLambda(wls_lambda);
	wls_filter->setSigmaColor(wls_sigma);

	while (running)
	{
		cam0.grabFrame();
		cam1.grabFrame();

		cam0.retrieveFrameBGR(_frame0);
		cam1.retrieveFrameBGR(_frame1);

		_frame0 = Camera::undistort(_frame0, calibrationData);
		_frame1 = Camera::undistort(_frame1, calibrationData);

		cvtColor(_frame0, left, COLOR_BGR2GRAY, CV_8U);
		cvtColor(_frame1, right, COLOR_BGR2GRAY, CV_8U);

		//Compute matches
		left_matcher->compute(left, right, left_disp);
		right_matcher->compute(right, left, right_disp);

		//Filter
		wls_filter->filter(left_disp, left, filtered_disp, right_disp);
		conf_map = wls_filter->getConfidenceMap();

		//get visualization
		ximgproc::getDisparityVis(filtered_disp, filtered_disp, 4.0);

        //Resize
        resize(left, left, Size(), 4.0, 4.0);
        resize(right, right, Size(), 4.0, 4.0);
        resize(filtered_disp, filtered_disp, Size(), 4.0, 4.0);

		imshow("Filtered Disparity", filtered_disp);
		imshow("Confidence Map", conf_map);

		imshow("Left", left);
		imshow("Right", right);
		if (waitKey(30) >= 0) break;
			cout << "frame " << std::setprecision(4) << cam0.getPositionFrame() << " @ " << cam0.getFrameRate() << " fps (" << cam0.getPositionSeconds() << " sec)" << endl;
	}
	return 0;
}
