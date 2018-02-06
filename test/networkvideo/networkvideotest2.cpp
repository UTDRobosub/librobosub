
#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <signal.h>
#include <opencv2/ximgproc.hpp>

#include <boost/asio/ip/tcp.hpp>

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
	        "{help h usage ? |         | print this message     }"
	        "{@mode          |         | 'send' or 'receive'    }"
	        "{p port         |2000     | port to send/listen to }"
					"{h host         |127.0.0.1| port to send on (not applicable for receive) }"
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

		//catch signal
		signal(SIGINT, catchSignal);
		
	int rows,cols;

    Size screenRes;
    Camera cam(camera);
    if(mode == MODE_SEND){
        if (!cam.isOpen()){
            cout<<"Camera failed to open."<<endl;
            return -1;
        }
        Size output = cam.setFrameSizeToMaximum();
        cout << output << endl;
        rows=output.height;
        cols=output.width;
        //assert(output.width == cols && output.height == rows);
    } else {
        screenRes = Util::getDesktopResolution();
    }
	
    cout<<rows<<" "<<cols<<endl;

    Mat frame1;

    if(mode == MODE_SEND){

        while(running){

            cam.retrieveFrameBGR(frame1);
			
            frame1=frame1.clone(); //make it continuous
			
			Drawing::text(frame1,
                String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
                Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );

            imshow("Sending Frame", frame1);

            if (waitKey(1) >= 0) break;
        }
    } else {

		FPS fps = FPS();
		int droppedFrames = 0;

        while(running){
			
            Mat *frame2 = RecvFrame(&udpr);
			
			Drawing::text(*frame2,
                String(Util::toStringWithPrecision(fps.fps())) + String(" FPS"),
                Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );
			Drawing::text(*frame2,
                String(Util::toStringWithPrecision(droppedFrames, 2)) + String(" Dropped Frames"),
                Point(16, 60), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );

            imshow("Receiving Frame", *frame2);

            if (waitKey(1) >= 0) break;
        }
	}

	return 0;
}
