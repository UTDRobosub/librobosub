
#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <robosub/networktcp.h>
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
	//Size frameSize = Size(parser.get<int>("vc"), parser.get<int>("vr"));
	
	//catch signal
	signal(SIGINT, catchSignal);
	
	int cols = 1280;
	int rows =  720;
	
	Size frameSize = Size(cols, rows);
	
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
	
	cols = frameSize.width;
	rows = frameSize.height;
	
	//load calibration data - run AFTER resolution set
	Camera::CalibrationData calibrationData = *Camera::loadCalibrationDataFromXML("../config/fisheye180_cameracalib_fisheye.xml", frameSize);
	
	int framelen = 3*rows*cols;
	int datalen = 16 + framelen;
	//4 bytes for cols
	//4 bytes for rows
	//framelen bytes for image data
	
	if(mode == MODE_SEND){
		
		char* senddata = (char*)malloc(datalen);
		Mat frame1;
		
		NetworkTcpServer server;
		
		cout<<"Binding to port."<<endl;
		server.bindToPort(port);
		
		cout<<"Accepting client."<<endl;
		server.acceptClient();
		
		cout<<"Connected."<<endl;
		
		while(running){
			
			cam->retrieveFrameBGR(frame1);
			
			*(int*)(senddata+ 0) = 1234567890;
			*(int*)(senddata+ 4) = cols;
			*(int*)(senddata+ 8) = rows;
			*(int*)(senddata+12) = 0;
			memcpy(senddata+16, frame1.data, framelen);
			
			server.sendBuffer(senddata, datalen);
			
			if (showDisplay) {
				Drawing::text(frame1,
					String(Util::toStringWithPrecision(cam->getFrameRate())) + String(" FPS"),
					Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				
				imshow("Sending Frame", frame1);
			}
			
			waitKey(50);
		}
	} else {
		
		FPS fps = FPS();
		Mat latestframe;
		
		int framesPerSecond;
		int bitsPerSecond;
		
		NetworkTcpClient client;
		
		cout<<"Connecting to server."<<endl;
		client.connectToServer((char*)addr.c_str(), port);
		
		cout<<"Connected."<<endl;
		
		while(running){
			char* recvdata = (char*)malloc(datalen);
			
			client.receiveBuffer(recvdata, datalen);
			
			int ver1 = *(int*)(recvdata+ 0);
			int cols = *(int*)(recvdata+ 4);
			int rows = *(int*)(recvdata+ 8);
			int ver2 = *(int*)(recvdata+12);
			
			//cout<<cols<<", "<<rows<<endl;
			
			if(ver1==1234567890){
				
				Mat bestframedraw(rows, cols, CV_8UC3, recvdata+16);
				
				fps.frame();
				
				framesPerSecond = fps.fps();
				bitsPerSecond = framesPerSecond*framelen*8;
				
				Drawing::text(bestframedraw,
					String(Util::toStringWithPrecision(framesPerSecond)) + String(" fps"),
					Point(16,64), Scalar(255,255,255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				Drawing::text(bestframedraw,
					String(Util::toStringWithPrecision(((float)bitsPerSecond)/1024.0f/1024.0f) + String(" Mbits/sec")),
					Point(16,16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
				);
				
				imshow("Latest Frame", bestframedraw);
			}
			
			waitKey(1);
		}
		
	}
	
	return 0;
}
