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

const int MODE_RECEIVE = 0;
const int MODE_SEND = 1;

int main(int argc, char** argv){

	const String keys =
		"{help ?         |         | print this message     }"
		"{@mode          |         | 'send' or 'receive'    }"
		"{p port         |8001     | port to send/listen to }"
		"{h host         |127.0.0.1| address to send to (not applicable for receive) }"
		"{vc cols        |1280     | image buffer columns }"
		"{vr rows        |720      | image buffer rows }"
		"{c cam camera   |0        | camera id }"
	;

	CommandLineParser parser(argc, argv, keys);
	parser.about("Network Video Transfer Test");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}
	if (!parser.has("@mode")) {
			cout << "Mode is required." << endl << endl;
			parser.printMessage();
			return 0;
	}
	if (!parser.check())
	{
		parser.printErrors();
		return 0;
	}

	int mode = parser.get<String>("@mode")[0] == 's' ? MODE_SEND : MODE_RECEIVE;
	int port = parser.get<int>("port");
	String addr = parser.get<String>("host");
	//int cols = parser.get<int>("cols");
	//int rows = parser.get<int>("rows");
	const int camera = parser.get<int>("camera");
    Size frameSize = Size(parser.get<int>("vc"), parser.get<int>("vr"));

	//catch signal
	signal(SIGINT, catchSignal);

	int rows,cols;

    Size screenRes;
    //Camera cam(camera);
    Camera *cam;
    if(mode == MODE_SEND){
		cam = new Camera(camera);
        if (!cam->isOpen()){
            cout<<"Camera failed to open."<<endl;
            return -1;
        }
        frameSize = cam->setFrameSize(frameSize);
        cout << frameSize << endl;
        //assert(output.width == cols && output.height == rows);
    } else {
        screenRes = Util::getDesktopResolution();
    }

    UDPS udps;
    UDPR udpr;
    if(mode == MODE_SEND)cout<<"initSend err "<<udps.initSend(port,addr)<<endl;
    else cout<<"initRecv err "<<udpr.initRecv(port)<<endl;

    Mat frame1;

    if(mode == MODE_SEND){

        while(running){

            cam->retrieveFrameBGR(frame1);

            frame1 = frame1.clone(); //make it continuous

			//frame1 = Camera::undistort(frame1, calibrationData);

            ImageTransform::flip(frame1, ImageTransform::FlipAxis::HORIZONTAL);

            SendFrame(&udps,&frame1);

			Drawing::text(frame1,
                String(Util::toStringWithPrecision(cam->getFrameRate())) + String(" FPS"),
                Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );

            imshow("Sending Frame", frame1);

            waitKey(1);
        }
    } else {

        //load calibration data - run AFTER resolution set
        Camera::CalibrationData calibrationData = *Camera::loadCalibrationDataFromXML("../config/fisheye180_cameracalibv2.xml", frameSize);

		FPS fps = FPS();
        Mat frame3;

        while(running){

            Mat* frame2 = RecvFrame(&udpr);

            if(frame2 == 0){
				cout<<"No frame data received yet."<<endl;
				robosub::Time::waitMillis(200);
            } else {
                fps.frame();

                frame3 = Camera::undistort(*frame2, calibrationData);

                Drawing::text(frame3,
					String(Util::toStringWithPrecision(fps.fps())) + String(" FPS"),
					Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);

				imshow("Receiving Frame", frame3);
            }

            waitKey(1);
        }
	}

	return 0;
}
