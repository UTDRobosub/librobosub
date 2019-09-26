#include "main.h"
#include <mutex>

const int VERIFICATION_CODE = 1234567890;
const int PORT[5] = {8500, 8501, 8502, 8503, 8504};
const String STEREO_ID = "usb-SHENZHEN_RERVISION_TECHNOLOGY_Stereo_Vision_2-video-index0";
mutex drawLock;

void catchSignal(int signal) {
    running = false;
}

void cameraThread(int port, String cameraName) {
    signal(SIGPIPE, catchSignal);

    Camera *cam;
    cam = new Camera(cameraName);
    if (!cam->isOpen()) {
        cout << "Camera failed to open." << endl;
        return;
    }
    Size frameSize = Size(1280, 720);

    frameSize = cam->setFrameSize(frameSize);
    cout << frameSize << endl;
    int cols = frameSize.width;
    int rows = frameSize.height;
    const int datalen = rows * cols * 3 + 16;
    NetworkTcpServer server;

    char *senddata = (char *) malloc(datalen);
    Mat frame1;

    cout << "Unbinding from port" << endl;
    server.unbindFromPort();

    cout << "Binding to port." << endl;
    server.bindToPort(port);

    cout << "Accepting client." << endl;
    server.acceptClient();

    cout << "Connected." << endl;

    float uploadBitsPerSecond = 0;

    while (running) {

        cam->retrieveFrameBGR(frame1);

        *(int *) (senddata + 0) = VERIFICATION_CODE;
        *(int *) (senddata + 4) = cols;
        *(int *) (senddata + 8) = rows;
        *(int *) (senddata + 12) = 0;
        memcpy(senddata + 16, frame1.data, rows * cols * 3);

        //test: break the frame into 100 segments and send one every 100 us (10 ms per frame)
        int segmentsize = datalen / 100;
        int numsegments = datalen / segmentsize + 1;
        for (int i = 0; i < numsegments; i++) {
            int ecode = server.sendBuffer(senddata + segmentsize * i, min(segmentsize, datalen - segmentsize * i));
            if (ecode != 0) {
                cout << "Send error: " << ecode << " " << strerror(ecode) << endl;
                break;
            }

            robosub::Time::waitMicros(100);
        }

        uploadBitsPerSecond = ((float) cam->getFrameRate()) * ((float) ((rows * cols * 3 + 16) * 8));

        waitKey(1);
    }
}

void startVideo() {
    String deviceIndexStr = Util::execCLI("ls /dev/video*");
    vector<String> deviceIndexes = Util::splitString(deviceIndexStr, '\n');

    String deviceNameStr = Util::execCLI("ls /dev/v4l/by-id/");
    vector<String> deviceNames = Util::splitString(deviceNameStr, '\n');
    bool stereoFound = false;
    //link camera id to port
    for (int i = 0; i < deviceNames.size(); i++) {
        if (STEREO_ID == deviceNames.at(i)) {
            stereoFound = true;
            cout << "Got stereo" << endl;
            continue;
        }

        String camIndex = Util::execCLI(String("readlink -f /dev/v4l/by-id/") + String(deviceNames.at(i)));
        camIndex.pop_back();
        //find the camIndex in deviceIndexes then swap it into i

        auto it = std::find(deviceIndexes.begin(), deviceIndexes.end(), camIndex);
        cout << "CAM INDEX: " << camIndex << endl;

        std::iter_swap(deviceIndexes.begin() + i, it);
        cout << i << " I did this " << deviceIndexes.at(i) << endl;
    }

    cout << "Left loop" << endl;

    //last two indexes in deviceIndexes will be for the stereo camera
    if (stereoFound) {
        //check that the second to last index is the one linked in v4l OR vice-versa (depends on when left/right once feeds are started)
    }
    const int numFeeds = (int) deviceIndexes.size();
    thread cameraThreads[numFeeds];
    for (int i = 0; i < numFeeds; i++) {
        String d = deviceIndexes.at(i);
        cameraThreads[i] = thread(cameraThread, PORT[i], d);
    }

    for (int i = 0; i < numFeeds; i++) {
        cameraThreads[i].join();
    }
}