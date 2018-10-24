#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <signal.h>
#include <opencv2/ximgproc.hpp>

#include "cvtest.cpp"

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
        "{d no-display   |false    | disable visualization (send only, faster) }"
		"{h host         |127.0.0.1| address to send to (send only) }"
		"{vc cols        |1280     | image buffer columns (send only)  }"
		"{vr rows        |720      | image buffer rows (send only)  }"
		"{c cam camera   |0        | camera id (send only) }"
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
    bool showDisplay = !parser.get<bool>("d");
	//int cols = parser.get<int>("cols");
	//int rows = parser.get<int>("rows");
	const int camera = parser.get<int>("camera");
    Size frameSize = Size(parser.get<int>("vc"), parser.get<int>("vr"));

	//catch signal
	signal(SIGINT, catchSignal);

	int rows,cols;

    Size screenRes;
    Camera *cam;
    if(mode == MODE_SEND){
		cam = new Camera(camera);
        if (!cam->isOpen()){
            cout<<"Camera failed to open."<<endl;
            return -1;
        }
        //frameSize = cam->setFrameSize(frameSize);
        frameSize = cam->getFrameSize();
        cout << frameSize << endl;
    } else {
        screenRes = Util::getDesktopResolution();
    }
    
    int receiveTimeoutMicroseconds = 1000;
    
    UDPS udps;
    UDPR udpr;
    if(mode == MODE_SEND)cout<<"initSend err "<<udps.initSend(port,addr)<<endl;
    else cout<<"initRecv err "<<udpr.initRecv(port, receiveTimeoutMicroseconds)<<endl;
	
    Mat frame1;
    
    if(mode==MODE_RECEIVE){
		cvtestInit();
    }

    //load calibration data - run AFTER resolution set
    Camera::CalibrationData calibrationData = *Camera::loadCalibrationDataFromXML("../config/fisheye180_cameracalib_fisheye.xml", frameSize);

    if(mode == MODE_SEND){

        while(running){

            cam->retrieveFrameBGR(frame1);
            
            //frame1 = frame1.clone(); //make it continuous
			//frame1 = Camera::undistort(frame1, calibrationData);
			//ImageTransform::rotate(frame1, 90);

            //ImageTransform::flip(frame1, ImageTransform::FlipAxis::HORIZONTAL);
			
			//ImageTransform::scale(frame1, 0.5);
			//frame1 = frame1.clone();
			
            SendFrame(udps, frame1);

            if (showDisplay) {
                Drawing::text(frame1,
                    String(Util::toStringWithPrecision(cam->getFrameRate())) + String(" FPS"),
                    Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
                );

                imshow("Sending Frame", frame1);
            }
            
            waitKey(100);
        }
    } else {

		FPS fps = FPS();
        Mat frame3;
        
        MovingAverage packetsPerFrameAvg(100); //moving average of 100 values
        
        int packetsPerFrame;
        int framesPerSecond;
        int packetsPerSecond;
        int bitsPerSecond;
        int packetSize = 8*512;
		
        NetworkVideoFrameReceiver* framerecv = new NetworkVideoFrameReceiver(udpr);
        
        while(running){
			
			int packetsLastFrame = framerecv->updateReceiveFrame();
            
            //cout<<"received "<<packetsLastFrame<<" packets"<<endl;
            
            if(!framerecv->isInitialized()) {
				cout<<"No frame data received yet."<<endl;
				robosub::Time::waitMillis(500);
            } else {
				//cout<<"draw frame"<<endl;
				
				Mat* frame2 = framerecv->getBestFrame();
				
				Mat frame3 = frame2->clone();

                cvtestDisplay(frame3);

                fps.frame();
                
                packetsPerFrameAvg.insertData(packetsLastFrame);
				packetsPerFrame = (int)packetsPerFrameAvg.getAverage();
                framesPerSecond = fps.fps();
                packetsPerSecond = packetsPerFrame*framesPerSecond;
                bitsPerSecond = packetsPerSecond*packetSize;
                
				Drawing::text(frame3,
					String(Util::toStringWithPrecision(framesPerSecond)) + String(" fps"),
					Point(16,64), Scalar(255,255,255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				Drawing::text(frame3,
					String(Util::toStringWithPrecision(packetsPerFrame) + String(" packets/frame")),
					Point(16,48), Scalar(255,255,255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				Drawing::text(frame3,
					String(Util::toStringWithPrecision(packetsPerSecond) + String(" packets/sec")),
					Point(16,32), Scalar(255,255,255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				Drawing::text(frame3,
					String(Util::toStringWithPrecision(((float)bitsPerSecond)/1024.0f/1024.0f) + String(" Mbits/sec")),
					Point(16,16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				
				imshow("Receiving Frame", frame3);
				
				waitKey(10);
            }

        }
        
        delete(framerecv);
	}

	return 0;
}
