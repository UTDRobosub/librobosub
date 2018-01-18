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

int main(int argc, char** argv){

	int mode;
	int port;
	string addr;

	cout<<"Enter mode (0 for receive, 1 for send): ";
	cin>>mode;

    cout<<"Enter port: ";
    cin>>port;

	if(mode==1){
        cin.ignore(100000,'\n');
        cout<<"Enter address to send to (leave blank for local): ";
        if(cin.peek()!='\n'){
            cin>>addr;
            cin.ignore(100000,'\n');
        }else{
            addr="127.0.0.1";
            cout<<addr<<endl;
        }
	}

	//catch signal
	signal(SIGINT, catchSignal);

    const int cols = 1280;
	const int rows = 720;

    Size screenRes;
    Camera cam(0);
    if(mode==1){
        if (!cam.isOpen()){
            cout<<"Camera failed to open."<<endl;
            return -1;
        }
        Size output = cam.setFrameSizeToMaximum();
        cout << output << endl;
        assert(output.width == cols && output.height == rows);
    }else{
        screenRes = Util::getDesktopResolution();
    }

    const int vals = 1;
    const int size = sizeof(uchar)*3;
    const int len=rows*cols*vals*size;
    char raw2[len];
    int len2;

    cout<<rows<<" "<<cols<<endl;

    UDPS udps;
    UDPR udpr;
    if(mode==1)cout<<"initSend err "<<udps.initSend(port,addr)<<endl;
    else cout<<"initRecv err "<<udpr.initRecv(port)<<endl;

    Mat frame1;

	while (running){
        if(mode==1){

            cam.retrieveFrameBGR(frame1);

            //ImageTransform::resize(frame1, Size(cols,rows));

            //cvtColor(frame1,frame1,COLOR_BGR2GRAY,CV_8U);

            Drawing::text(frame1,
                String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
                Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );

            //frame1=frame1.clone(); //make it continuous

            cout<<"send err "<<udps.send(len,(char*)frame1.data)<<endl;

            imshow("Frame1", frame1);

        }else{

            cout<<"recv err "<<udpr.recv(len,len2,raw2)<<endl;

            Mat frame2(rows,cols,CV_8UC3,raw2);

            ImageTransform::scale(frame2, Size(cols,rows));

            imshow("Frame2", frame2);

        }

		if (waitKey(30) >= 0) break;
	}

	return 0;
}
