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
    char raw2[len*2];
    char raw2final[len];
    int len2;
    int len2total;
    int raw2start;

    cout<<rows<<" "<<cols<<endl;

    UDPS udps;
    UDPR udpr;
    if(mode==1)cout<<"initSend err "<<udps.initSend(port,addr)<<endl;
    else cout<<"initRecv err "<<udpr.initRecv(port)<<endl;

    Mat frame1;

    if(mode==1){

        while(running){

            cam.retrieveFrameBGR(frame1);

            //ImageTransform::resize(frame1, Size(cols,rows));

            //cvtColor(frame1,frame1,COLOR_BGR2GRAY,CV_8U);

            Drawing::text(frame1,
                String(Util::toStringWithPrecision(cam.getFrameRate())) + String(" FPS"),
                Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );

            frame1=frame1.clone(); //make it continuous

            for(int i=0; i<len; i++){
                ((char*)frame1.data)[i]=((char*)frame1.data)[i] & 0xFC; //make low-order 2 bits of every byte 0 to identify the middle of a frame
            }
            ((char*)frame1.data)[0]=((char*)frame1.data)[0] | 0x01; //make the first bit identify the start of a frame
            ((char*)frame1.data)[len-1]=((char*)frame1.data)[len-1] | 0x02; //make the last bit identify the end of a frame

            int senderr=udps.send(len,(char*)frame1.data);
            if(senderr!=0){
                cout<<"send err "<<senderr<<endl;
            }

            imshow("Frame1-changed", frame1);

            if (waitKey(1) >= 0) break;
        }
    }else{

        while(running){

            int recverr=udpr.recv(len*2-len2total,len2,raw2+len2total);
            if(recverr!=0){
                cout<<"recv err "<<recverr<<endl;
            }

            len2total+=len2;

            if(len2total==len*2){
                len2total=0;
                cout<<"overflow"<<endl;
            }else{

                for(int i=len2total-len2; i<len2total; i++){
                    if(raw2[i] & 0x01){
                        raw2start=i;
                        cout<<"start "<<i<<endl;
                    }
                    if(raw2[i] & 0x02){
                        if(raw2start!=-1 && i-raw2start==len-1){
                            memcpy(raw2final,raw2+raw2start,i-raw2start);
                            memcpy(raw2,raw2+i+1,len2total-i-1);
                            len2total=len2total-i-1;
                            raw2start=-1;
                            cout<<"end "<<i<<endl;
                        }else{
                            len2total=0;
                        }
                    }
                }
            }

            //if(len2total>=len)len2total-=len;

            Mat frame2(rows,cols,CV_8UC3,raw2final);

            ImageTransform::scale(frame2, Size(cols,rows));

            imshow("Frame2", frame2);

            if (waitKey(1) >= 0) break;
        }
	}

	return 0;
}
