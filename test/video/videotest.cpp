#include <opencv2/opencv.hpp>
#include <cvlib/cvlib.h>

using namespace std;
using namespace cvlib;

template <typename T>
std::string to_string_with_precision(const T value, const int n = 4)
{
	std::ostringstream out;
	out << std::setprecision(n) << value;
	return out.str();
}

int main(int argc, char** argv)
{
	Camera cam = Camera(0);

	if (!cam.isOpen()) return -1;

	namedWindow("edges", 1);
	Mat frame;
	cout << cam.setFrameSizeToMaximum() << endl;
	for (;;)
	{
		cam.retrieveFrameGrey(frame); // get a new frame from camera
		GaussianBlur(frame, frame, Size(7, 7), 1.5, 1.5);
		Canny(frame, frame, 0, 30, 3);
		imshow("edges", frame);
		if (waitKey(30) >= 0) break;
		cout << "frame " << cam.getPositionFrame() << " @ " << cam.getFrameRate() << " fps (" << cam.getPositionSeconds() << " sec)" << endl;
	}
	return 0;
}