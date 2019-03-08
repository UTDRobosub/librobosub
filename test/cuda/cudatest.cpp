#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#ifdef HAVE_OPENCV_CUDAIMGPROC

#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"

#endif

#include <robosub/robosub.h>

using namespace cv;
using namespace robosub;

int main (int argc, char* argv[])
{
#ifdef HAVE_OPENCV_CUDAIMGPROC
    try
    {
        cout << cv::getBuildInformation() << endl;

        cv::Mat matLeft, matRight, disparity;
        cuda::GpuMat gpuMatLeft, gpuMatRight, gpuMatDisparity;

        Camera camLeft = Camera(1);
        Camera camRight = Camera(2);
        camLeft.setFrameSizeToMaximum();
        camRight.setFrameSizeToMaximum();

        cv::namedWindow("Left", WINDOW_AUTOSIZE);
        cv::namedWindow("Right", WINDOW_AUTOSIZE);
        cv::namedWindow("Disparity", WINDOW_AUTOSIZE);

        Ptr<cuda::CannyEdgeDetector> canny = cuda::createCannyEdgeDetector(40.0, 60.0, 3, true);

        int ndisp = 88;
        Ptr<cuda::StereoBM> bm = cuda::createStereoBM(ndisp);

        while(true) {
            if (!camLeft.isOpen())
                throw runtime_error("Left camera not open");
            if (!camRight.isOpen())
                throw runtime_error("Right camera not open");

            camRight.grabFrame();
            camLeft.grabFrame();
            camRight.getGrabbedFrame(matRight);
            camLeft.getGrabbedFrame(matLeft);


            cv::imshow("Left", matLeft);
            cv::imshow("Right", matRight);

            gpuMatLeft.upload(matLeft);
            gpuMatRight.upload(matRight);

            cuda::cvtColor(gpuMatLeft, gpuMatLeft, COLOR_BGR2GRAY);
            cuda::cvtColor(gpuMatRight, gpuMatRight, COLOR_BGR2GRAY);
            bm->compute(gpuMatLeft, gpuMatRight, gpuMatDisparity);

            gpuMatDisparity.download(disparity);


            cv::imshow("Disparity", disparity);

//            cuda::cvtColor(gpuMatIn, gpuMatOut, COLOR_BGR2GRAY);
//            cuda::equalizeHist(gpuMatOut, gpuMatOut);
//            canny->detect(gpuMatOut, gpuMatOut);
//            cuda::cvtColor(gpuMatOut, gpuMatOut, COLOR_GRAY2BGR);
//            cuda::bitwise_not(gpuMatOut,gpuMatOut);
//            cuda::bitwise_and(gpuMatIn, gpuMatOut, gpuMatOut);
//            cuda::gammaCorrection(gpuMatOut, gpuMatOut);

//            cuda::threshold(gpuMatIn, gpuMatOut, 128.0, 255.0, THRESH_BINARY);

            if (cv::waitKey(1) > 0)
                break;
        }
    }
    catch(const cv::Exception& ex)
    {
        std::cout << "Error: " << ex.what() << std::endl;
    }
    return 0;
#endif
    cout << cv::getBuildInformation() << endl;
    cout << "OpenCV not built with CUDA support." << endl;
    return 1;
}