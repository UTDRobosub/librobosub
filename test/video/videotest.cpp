#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <signal.h>

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

	Camera cam = Camera(0);

	if (!cam.isOpen()) return -1;

	Scalar mu, sigma;

	Mat frame, output, canny, final;
	//cout << cam.setFrameSizeToMaximum() << endl;
	Size screenRes = Util::getDesktopResolution();

	while (running)
	{
		cam.retrieveFrameBGR(frame);

		//Flip image
		ImageTransform::flip(frame, ImageTransform::FlipAxis::VERTICAL);

		//Convert frame to grayscale
		cvtColor(frame, output, COLOR_BGR2GRAY, CV_8U);

		//Compute standard deviation for image
		meanStdDev(output, mu, sigma);

		//Run adaptive algorithm
		//adaptiveThreshold(output, output, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 15, 8);

		//medianBlur(output, output, 5);

		//Remove small noise
		int erosion_size = 3;
		Mat element = getStructuringElement(MORPH_ELLIPSE,
			Size(2 * erosion_size + 1, 2 * erosion_size + 1),
			Point(erosion_size, erosion_size));
		//dilate(output, output, element);
		erode(output, output, element);
		dilate(output, output, element);

		//Threshold
		//adaptiveThreshold(output, output, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 15, 8);
		Canny(output, output, mu.val[0] - 2.0 * sigma.val[0], mu.val[0] + 0.0 * sigma.val[0]);
		dilate(output, output, element);

		//Find contours
		vector<vector<Point> > contours;
		vector<Vec4i> hierarchy;
		findContours(output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);
		for (unsigned int i = 0; i<contours.size(); i++)
			if (hierarchy[i][3] >= 0)   //has parent, inner (hole) contour of a closed edge (looks good)
				drawContours(frame, contours, i, Scalar(0, 0, 0), 4, 8);

		//Run canny detection with +- 1 std dev of random values
		//Canny(output, output, mu.val[0] - 0.66 * sigma.val[0], mu.val[0] + 1.33 * sigma.val[0]);

		//Draw FPS text
		Drawing::text(frame,
			String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
			Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5);
		Drawing::text(output,
			String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
			Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5);
		Drawing::text(output,
			String("Mu = ") + Util::toStringWithPrecision(mu.val[0]) + String("  Sigma = ") + Util::toStringWithPrecision(sigma.val[0]),
			Point(120, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5);

		ImageTransform::scale(frame, screenRes - Size(0, 128));
		ImageTransform::scale(output, screenRes - Size(0, 128));

		cvtColor(frame, frame, COLOR_BGR2GRAY);

		imshow("Output", frame);
		if (waitKey(30) >= 0) break;
		cout << "frame " << std::setprecision(4) << cam.getPositionFrame() << " @ " << cam.getFrameRate() << " fps (" << cam.getPositionSeconds() << " sec)" << endl;
	}
	return 0;
}
