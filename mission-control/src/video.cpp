#include "main.h"

using namespace std;
using namespace robosub;

const int VERIFICATION_CODE = 1234567890;
String ADDR = "192.168.1.2";
const int NUMFEEDS = 4;
int PORT[4] = {8500, 8502, 8503, 8504};
const int TIMEOUT_LIMIT = 50;
//void catchSignal(int signal) {
//    running = false;
//}

void drawFrame(int rows, int cols, char *framedata, float framesPerSecond, float bitsPerSecond, int port) {
    int framedatalen = rows * cols * 3;

    Mat bestframedraw(rows, cols, CV_8UC3, framedata);

    Drawing::text(bestframedraw,
                  String(Util::toStringWithPrecision(framesPerSecond)) + String(" fps"),
                  Point(16, 48), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
    );
    Drawing::text(bestframedraw,
                  String(Util::toStringWithPrecision((bitsPerSecond) / 1024.0f / 1024.0f) + String(" Mbps")),
                  Point(16, 16), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 0.5
    );

    imshow(String("Port ") + String(Util::toStringWithPrecision(port, 0)), bestframedraw);
}
void drawError(int rows, int cols, int port) {
    int framedatalen = rows * cols * 3;

    Mat bestframedraw(rows, cols, CV_8UC3, Scalar(32, 32, 32));

    Drawing::text(bestframedraw,
                  String("Not connected"),
                  Point(16, 72), Scalar(255, 255, 255), Drawing::Anchor::BOTTOM_LEFT, 2.0
    );

    imshow(String("Port ") + String(Util::toStringWithPrecision(port, 0)), bestframedraw);
}

void cameraThread(int port){
    int rows = 720;
    int cols = 1280;
    FPS fps = FPS();

    float framesPerSecond;
    float bitsPerSecond;

    while(running)
    {
        NetworkTcpClient client;

        cout << "Connecting to server." << endl;
        int err = client.connectToServer((char *) ADDR.c_str(), port);
        if (err) {
            cout << "Connection Error " << err << ": " << strerror(err) << endl;
            drawError(rows, cols, port);
            waitKey(1);
        } else {
            cout << "Connected." << endl;

            char *framedata = nullptr;

            char *headerdata = (char *) malloc(16);

            int waitingOnRestOfFrame = 0;
            int framedatalen = 0;

            int previousDataRemaining = 0;
            int timeoutCounter = 0;
            while (running) {
                if (waitingOnRestOfFrame == 0) {
                    int headerlen = client.receiveBuffer(headerdata, 16);

                    if (headerlen == 16) {
                        int ver1 = *(int *) (headerdata + 0);
                        cols = *(int *) (headerdata + 4);
                        rows = *(int *) (headerdata + 8);
                        int none = *(int *) (headerdata + 12);

                        //cout<<cols<<" "<<rows<<endl;

                        if (ver1 == VERIFICATION_CODE) {

                            fps.frame();

                            framesPerSecond = (float) fps.fps();
                            bitsPerSecond = (float) (framesPerSecond * (framedatalen + 16) * 8);

                            framedatalen = rows * cols * 3;

                            if (framedata != nullptr) {
                                free(framedata);
                            }

                            framedata = (char *) malloc(framedatalen);

                            int recvdatalen = client.receiveBuffer(framedata, framedatalen);

                            if (recvdatalen >= 0) {
                                waitingOnRestOfFrame = framedatalen - recvdatalen;
                                cout<<"waiting on 2 "<<waitingOnRestOfFrame<<endl;

                                if (waitingOnRestOfFrame == 0) {
                                    drawFrame(rows, cols, framedata, framesPerSecond, bitsPerSecond, port);
                                }
                            }
                        }
                    }
                } else {
                    int recvdatalen = client.receiveBuffer(framedata + (framedatalen - waitingOnRestOfFrame),
                                                           waitingOnRestOfFrame);

                    if (recvdatalen >= 0) {
                        waitingOnRestOfFrame = waitingOnRestOfFrame - recvdatalen;

                        cout<<"waiting on "<<waitingOnRestOfFrame<<endl;

                        if(previousDataRemaining == waitingOnRestOfFrame){
                            timeoutCounter++;
                            if(timeoutCounter > TIMEOUT_LIMIT){
                                break;
                            }
                        } else {
                            timeoutCounter = 0;
                        }
                        previousDataRemaining = waitingOnRestOfFrame;
                        if (waitingOnRestOfFrame == 0) {
                            drawFrame(rows, cols, framedata, framesPerSecond, bitsPerSecond, port);
                        }

                    }
                }

                waitKey(1);
            }
        }
    }
}



void video() {
//    signal(SIGINT, catchSignal);
    thread cameraThreads[NUMFEEDS];

    for (int i = 0; i < NUMFEEDS; i++) {
        cameraThreads[i] = thread(cameraThread, PORT[i]);
    }

    for(int i = 0; i < NUMFEEDS; i++) {
        cameraThreads[i].join();
    }
}

