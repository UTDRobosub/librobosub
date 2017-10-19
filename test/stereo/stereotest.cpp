#include <opencv2/opencv.hpp>
#include <cvlib/cvlib.h>
#include <signal.h>
#include <opencv2/ximgproc.hpp>

using namespace std;
using namespace cvlib;

bool running = true;

void catchSignal(int signal) {
	running = false;
}

int main(int argc, char** argv)
{
	//catch signal
	signal(SIGINT, catchSignal);

	Camera cam0 = Camera(1);
	Camera cam1 = Camera(2);

	if (!cam0.isOpen()) return -1;
	if (!cam1.isOpen()) return -1;

	cam1.setFrameSize(cam0.getFrameSize());

	Mat _frame0, _frame1, left, right, left_disp, right_disp, filtered_disp;
	//cout << cam0.setFrameSizeToMaximum() << endl;
	left.create(cam0.getFrameSize(), CV_8U);
	right.create(cam1.getFrameSize(), CV_8U);

	int max_disp = 160; //maximum disparity
	double vis_mult = 12.0; //output disparity scale factor
	double wls_lambda = 8000.0;
	double wls_sigma = 1.5;
	int wsize = 5;

	//Create confidence map
	Mat conf_map = Mat(left.rows, left.cols, CV_8U);
	conf_map = Scalar(255);

	Rect ROI;
	Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;

	while (running)
	{
		cam0.grabFrame();
		cam1.grabFrame();

		cam0.retrieveFrameRGBA(_frame0);
		cam1.retrieveFrameRGBA(_frame1);

		_frame0.copyTo(left);
		_frame1.copyTo(right);

		max_disp /= 2;
		if (max_disp % 16 != 0)
			max_disp += 16 - (max_disp % 16);
		resize(left, left, Size(), 0.5, 0.5);
		resize(right, right, Size(), 0.5, 0.5);

		//Prepare matchers
		Ptr<StereoBM> left_matcher = StereoBM::create(max_disp, wsize);
		wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
		Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);
		cvtColor(left, left, COLOR_BGR2GRAY);
		cvtColor(right, right, COLOR_BGR2GRAY);
		/*Ptr<StereoSGBM> left_matcher = StereoSGBM::create(0, max_disp, wsize);
		left_matcher->setBlockSize(wsize);
		left_matcher->setP1(24 * wsize*wsize);
		left_matcher->setP2(96 * wsize*wsize);
		left_matcher->setPreFilterCap(63);
		left_matcher->setMode(StereoSGBM::MODE_SGBM_3WAY);
		wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
		Ptr<StereoMatcher> right_matcher = ximgproc::createRightMatcher(left_matcher);*/
		//Compute matches
		left_matcher->compute(left, right, left_disp);
		right_matcher->compute(right, left, right_disp);
		
		//Filter
		wls_filter->setLambda(wls_lambda);
		wls_filter->setSigmaColor(wls_sigma);
		wls_filter->filter(left_disp, left, filtered_disp, right_disp);
		conf_map = wls_filter->getConfidenceMap();

		//Resize
		resize(left, left, Size(), 2.0, 2.0);
		resize(right, right, Size(), 2.0, 2.0);
		resize(filtered_disp, filtered_disp, Size(), 2.0, 2.0);

		ximgproc::getDisparityVis(filtered_disp, filtered_disp, 10.0);

		imshow("Filtered Disparity", filtered_disp);
		imshow("Confidence Mao", conf_map);

		imshow("CAM 0", left);
		imshow("CAM 1", right);
		if (waitKey(30) >= 0) break;
		//cout << "frame " << std::setprecision(4) << cam0.getPositionFrame() << " @ " << cam0.getFrameRate() << " fps (" << cam0.getPositionSeconds() << " sec)" << endl;
	}
	return 0;
}