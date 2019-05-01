#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>


#ifdef HAVE_CUDA
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"

using namespace std;
using namespace robosub;

int main() {
    Camera leftCam = Camera(1);
    Camera rightCam = Camera(2);

    leftCam.setFrameSize(Size(320, 240));
    Size frameSize = leftCam.getFrameSize();
    rightCam.setFrameSize(frameSize);

    namedWindow("Left original");
    namedWindow("Right original");
    namedWindow("Left rectified");
    namedWindow("Right rectified");

    if (!leftCam.isOpen() || !rightCam.isOpen()) return -1;

    Mat left, right, leftRect, rightRect;
    Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, R1, P1, R2, P2, rmap11, rmap12, rmap21, rmap22;

    //load some data
    FileStorage fs;
    fs.open("intrinsics.xml", FileStorage::READ);
    fs["M1"] >> cameraMatrix1;
    fs["M2"] >> cameraMatrix2;
    fs["D1"] >> distCoeffs1;
    fs["D2"] >> distCoeffs2;
    fs.release();

    fs.open("extrinsics.xml", FileStorage::READ);
    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs.release();

    //prepare parameters for remap
    initUndistortRectifyMap(cameraMatrix1, distCoeffs1, R1, P1, frameSize, CV_16SC2, rmap11, rmap12);
    initUndistortRectifyMap(cameraMatrix2, distCoeffs2, R2, P2, frameSize, CV_16SC2, rmap21, rmap22);

    while(true) {
        leftCam.grabFrame();
        rightCam.grabFrame();

        leftCam.getGrabbedFrame(left);
        rightCam.getGrabbedFrame(right);

        //remap on frame
        remap(left, leftRect, rmap11, rmap12, INTER_LINEAR);
        remap(right, rightRect, rmap21, rmap22, INTER_LINEAR);



        imshow("Left original", left);
        imshow("Right original", right);
        imshow("Left rectified", leftRect);
        imshow("Right rectified", rightRect);

        if (waitKey(1) >= 0) break;
    }
}

#else

int main() {
    cout << "CUDA not compiled into OpenCV! You might want to do that." << endl;
}

#endif