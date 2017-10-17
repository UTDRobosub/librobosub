#include <opencv2/opencv.hpp>
#include <cvlib/cvlib.h>
#include <signal.h>

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

	Camera cam0 = Camera(0);
	//Camera cam1 = Camera(1);

	if (!cam0.isOpen()) return -1;
	//if (!cam1.isOpen()) return -1;

	Mat frame0, frame1, disp;
	//cout << cam0.setFrameSizeToMaximum() << endl;

	Ptr<StereoSGBM> sgbm = StereoSGBM::create();

	int sgbmWinSize = 3;
	sgbm->setBlockSize(sgbmWinSize);

	int cn = 1; //channels on image

	//sgbm->setP1(8 * cn*sgbmWinSize*sgbmWinSize);
	//sgbm->setP2(32 * cn*sgbmWinSize*sgbmWinSize);
	//sgbm->setMinDisparity(0);
	//int numberOfDisparities = 4;
	//sgbm->setNumDisparities(16 * numberOfDisparities); //must be multiple of 16
	//sgbm->setUniquenessRatio(10);
	//sgbm->setSpeckleWindowSize(100);
	//sgbm->setSpeckleRange(32);
	//sgbm->setDisp12MaxDiff(1);
	//sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

	while (running)
	{
		cam0.grabFrame();
		//cam1.grabFrame();

		cam0.retrieveFrameGrey(frame0);
		//cam1.retrieveFrameGrey(frame1);

		//sgbm->compute(frame0, frame1, disp);
		//disp.convertTo(disp, CV_8U, 255 / (numberOfDisparities*256.0));
		//normalize(disp, disp, 0, 255, CV_MINMAX, CV_8U);
		
		imshow("CAM 0", frame0);
		//imshow("CAM 1", frame1);
		//imshow("Normalized Disparity", disp);
		if (waitKey(30) >= 0) break;
		cout << "frame " << std::setprecision(4) << cam0.getPositionFrame() << " @ " << cam0.getFrameRate() << " fps (" << cam0.getPositionSeconds() << " sec)" << endl;
	}
	return 0;
}