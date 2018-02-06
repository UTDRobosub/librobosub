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
		const int cols = parser.get<int>("cols");
		const int rows = parser.get<int>("rows");
		const int camera = parser.get<int>("camera");

		//catch signal
		signal(SIGINT, catchSignal);

    Size screenRes;
    Camera cam(camera);
    if(mode == MODE_SEND){
        if (!cam.isOpen()){
            cout<<"Camera failed to open."<<endl;
            return -1;
        }
        Size output = cam.setFrameSizeToMaximum();
        cout << output << endl;
        assert(output.width == cols && output.height == rows);
    } else {
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
    if(mode == MODE_SEND)cout<<"initSend err "<<udps.initSend(port,addr)<<endl;
    else cout<<"initRecv err "<<udpr.initRecv(port)<<endl;

    Mat frame1;

    if(mode == MODE_SEND){

        while(running){

            cam.retrieveFrameBGR(frame1);

            //ImageTransform::resize(frame1, Size(cols,rows));

            //cvtColor(frame1,frame1,COLOR_BGR2GRAY,CV_8U);

            frame1=frame1.clone(); //make it continuous

            /*
            for(int i=0; i<len; i++){
                ((char*)frame1.data)[i]=((char*)frame1.data)[i] & 0xFC; //make low-order 2 bits of every byte 0 to identify the middle of a frame
            }
            ((char*)frame1.data)[0]=((char*)frame1.data)[0] | 0x01; //make the first bit identify the start of a frame
            ((char*)frame1.data)[len-1]=((char*)frame1.data)[len-1] | 0x02; //make the last bit identify the end of a frame

            int senderr=udps.send(len,(char*)frame1.data);
            if(senderr!=0){
                cout<<"send err "<<senderr<<endl;
            }*/

            SendFrame(&udps,&frame1);

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

            /*
            int recverr=udpr.recv(len*2-len2total,len2,raw2+len2total);
            if(recverr!=0){
                cout<<"recv err "<<recverr<<endl;
            }

            len2total+=len2;

            if(len2total==len*2){
                len2total=0;
                cout<<"overflow"<<endl;
            } else {

                for(int i=len2total-len2; i<len2total; i++){
                    if(raw2[i] & 0x01){
                        raw2start=i;
                        cout<<"start "<<i<<endl;
                    }
                    if(raw2[i] & 0x02){
                        if(raw2start != -1 && i-raw2start == len - 1){
														//complete frame received

                            memcpy(raw2final,raw2+raw2start,i-raw2start);
                            // memcpy(raw2,raw2+i+1,len2total-i-1);
                            len2total=len2total-i-1;

                            raw2start=-1;
                            cout<<"end "<<i<<endl;
														fps.frame();
                        }else{
														//drop frame
                            len2total=0;
														droppedFrames++;
                        }
                    }
                }
            }

            //if(len2total>=len)len2total-=len;

            Mat frame2(rows,cols,CV_8UC3,raw2final);*/

            Mat *frame2 = RecvFrame(&udpr);

						Drawing::text(*frame2,
                String(Util::toStringWithPrecision(fps.fps())) + String(" FPS"),
                Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );
						Drawing::text(*frame2,
                String(Util::toStringWithPrecision(droppedFrames, 2)) + String(" Dropped Frames"),
                Point(16, 60), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
            );

						//ImageTransform::scale(frame2, Size(cols,rows));

            imshow("Receiving Frame", *frame2);

            if (waitKey(1) >= 0) break;
        }
	}

	return 0;
}
