#include <opencv2/opencv.hpp>
#include <robosub/robosub.h>
#include <fstream>

using namespace std;
using namespace robosub;

int main(int argc, char **argv) {
    //create window names
    namedWindow("Left");
    namedWindow("Right");

    //init cameras
    Camera leftCam = Camera(1);
    Camera rightCam = Camera(2);
    leftCam.setFrameSize(Size(320, 240));
    rightCam.setFrameSize(leftCam.getFrameSize());

    Mat left, right, leftDisplayed, rightDisplayed;

    if (!leftCam.isOpen()) return -1;
    if (!rightCam.isOpen()) return -2;

    int fileNumber = 1;
    while (Util::fileExists("left" + to_string(fileNumber) + ".jpg")) {
        fileNumber++;
    }

    while (true) {
        leftCam.grabFrame();
        rightCam.grabFrame();

        leftCam.getGrabbedFrame(left);
        rightCam.getGrabbedFrame(right);

        left.copyTo(leftDisplayed);
        right.copyTo(rightDisplayed);

        ImageTransform::scale(leftDisplayed, 4);
        ImageTransform::scale(rightDisplayed, 4);

        imshow("Left", leftDisplayed);
        imshow("Right", rightDisplayed);

        //if spacebar pressed, take picture
        int key = waitKey(1);
        if (key == 32) {
            //save to disk
            imwrite("left" + to_string(fileNumber) + ".jpg", left);
            imwrite("right" + to_string(fileNumber) + ".jpg", right);

            cout << "Saved image number " << fileNumber << endl;

            while (Util::fileExists("left" + to_string(fileNumber) + ".jpg")) {
                fileNumber++;
            }

        } else if (key == 119) { //w key (write and save)
            ofstream xml;
            xml.open("stereo_calib.xml");

            xml << "<?xml version=\"1.0\"?>\n"
                   "<opencv_storage>\n"
                   "<imagelist>\n";
            for (int i = 1; i < fileNumber; i++) {
                xml << "left" << i << ".jpg" << endl;
                xml << "right" << i << ".jpg" << endl;
            }

            xml << "</imagelist>\n"
                   "</opencv_storage>";

            cout << "Wrote XML file" << endl;
        } else if (key == 113) { //q key (quit)
            cout << "Bye!" << endl;
            break;
        }
    }
}