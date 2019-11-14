//
// Created by robosub on 11/13/19.
//

#include "shapecalibimagecapture.h"

using namespace std;

namespace robosub {
    int main(int argc, char **argv) {
        // TODO: Fix this to compile. The code should be correct, but the project build is having issues
        struct sigaction action = getSigaction();

        sigaction(SIGINT, &action, NULL);

        // cam.setFrameSize(Size(1280, 720));
        Camera::CalibrationData calibrationData = *Camera::loadCalibrationDataFromXML(
                "../config/fisheye_cameracalib.xml",
                std::move(Size(1280, 720)));
        Mat input, output;
        Camera cam = Camera("/dev/video0");
        ShapeFinder shapeFinder(calibrationData);
        ShapeFindResult result;

        createTuningWindow(shapeFinder);


        while (running) {
            cam.retrieveFrameBGR(input);
            imshow("Input", input);

            shapeFinder.processFrame(input, result);
            displayShapes(result, input);
            displayShapeCountUi(result, input);
            displayFrameCount(cam.getFrameRate(), input);

            ImageTransform::scale(input, .5);

            ImageTransform::scale(input, .5);

            imshow("Output", input);

            int keyPress = waitKey(1);

            if (keyPress == 32) { //take a picture and save values





            } else if (keyPress >= 65 && keyPress <= 122)
                break;
        }

        return 0;
    }

}


}