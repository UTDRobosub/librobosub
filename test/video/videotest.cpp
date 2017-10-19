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

	Camera cam = Camera(0);

	if (!cam.isOpen()) return -1;

	Mat frame;
	//cout << cam.setFrameSizeToMaximum() << endl;

	while (running)
	{
		cam.retrieveFrameGrey(frame);

		ImageTransform::flip(frame, ImageTransform::FlipAxis::VERTICAL);

		Drawing::text(frame, 
			String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
			Point(25, 25), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5);
		
		imshow("Camera", frame);
		if (waitKey(30) >= 0) break;
		cout << "frame " << std::setprecision(4) << cam.getPositionFrame() << " @ " << cam.getFrameRate() << " fps (" << cam.getPositionSeconds() << " sec)" << endl;
	}
	return 0;
}
