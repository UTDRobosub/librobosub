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

int main(int argc, char** argv)
{
    int mode;
    cin>>mode;

	//catch signal
	signal(SIGINT, catchSignal);

    Camera cam(0);
    if(mode==1){
        if (!cam.isOpen()){
            cout<<"Camera failed to open"<<endl;
            return -1;
        }
    }

	int rows = 480;
    int cols = 640;
    const int vals = 1;
    int size = sizeof(uchar);
    int len=rows*cols*vals*size/4+4;
    char raw[len];
    char raw2[len];
    int len2;

    cout<<rows<<" "<<cols<<endl;

    UDPS udps;
    UDPR udpr;
    if(mode==1)cout<<"initSend err "<<udps.initSend(20202,"localhost")<<endl;
    else cout<<"initRecv err "<<udpr.initRecv(20202)<<endl;

	while (running){
        if(mode==1){

            cam.grabFrame();

            Mat _frame;
            cam.retrieveFrameBGR(_frame);

            Mat frame1;
            resize(_frame,frame1,Size(),0.5,0.5);

            cvtColor(frame1,frame1,COLOR_BGR2GRAY,CV_8U);

            frame1=frame1.clone(); //make it continuous
            memcpy(raw,frame1.data,len);

            cout<<"send err "<<udps.send(len,raw)<<endl;

            imshow("Frame1", frame1);

        }else{

            cout<<"recv err "<<udpr.recv(len,len2,raw2)<<endl;

            Mat frame2(rows/2,cols/2,CV_8UC1,raw2);

            resize(frame2,frame2,Size(),2,2);

            imshow("Frame2", frame2);
        }

		if (waitKey(30) >= 0) break;
	}

	return 0;
}
