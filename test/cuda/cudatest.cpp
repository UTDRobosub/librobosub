#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#ifdef HAVE_OPENCV_CUDAIMGPROC

#include "opencv2/cudafilters.hpp"
#include "opencv2/cudaimgproc.hpp"

#endif

#include <robosub/robosub.h>
#include <opencv2/ximgproc/disparity_filter.hpp>

using namespace cv;
using namespace robosub;

int main(int argc, char *argv[]) {
#ifdef HAVE_OPENCV_CUDAIMGPROC
    try
    {

        if (cuda::getCudaEnabledDeviceCount() < 1)
            throw runtime_error("No CUDA devices found");

        //use last device
        cuda::setDevice(cuda::getCudaEnabledDeviceCount() - 1);

        cout << "CUDA Device: " << cuda::getDevice() << " (count: " << cuda::getCudaEnabledDeviceCount() << ")" << endl;
        cout << "CUDA Device Supports Compute 5.0: " << cuda::deviceSupports(cuda::FEATURE_SET_COMPUTE_50) << endl;


        cv::Mat matLeft, matRight, disparityLeft, disparityRight, disparityFiltered;
        cuda::GpuMat gpuMatLeft, gpuMatRight, gpuMatDisparityLeft, gpuMatDisparityRight;

        Camera camLeft = Camera(2);
        Camera camRight = Camera(1);

        cout << camLeft.setFrameSize(cv::Size(320, 240)) << endl;
        Size frameSize = camLeft.getFrameSize();
        camRight.setFrameSize(frameSize);

        cv::namedWindow("Left", WINDOW_AUTOSIZE);
        cv::namedWindow("Right", WINDOW_AUTOSIZE);
        cv::namedWindow("Disparity", WINDOW_AUTOSIZE);

        Ptr<cuda::CannyEdgeDetector> canny = cuda::createCannyEdgeDetector(40.0, 60.0, 3, true);

        //load calibration data

        Mat R1, R2, P1, P2, Q;
        Mat K1, K2, R;
        Vec3d T;
        Mat D1, D2;

        cv::FileStorage fs1("../config/stereo_full.xml", cv::FileStorage::READ);
        fs1["K1"] >> K1;
        fs1["K2"] >> K2;
        fs1["D1"] >> D1;
        fs1["D2"] >> D2;
        fs1["R"] >> R;
        fs1["T"] >> T;

        fs1["R1"] >> R1;
        fs1["R2"] >> R2;
        fs1["P1"] >> P1;
        fs1["P2"] >> P2;
        fs1["Q"] >> Q;

        cv::Mat lmapx, lmapy, rmapx, rmapy;
//        FileStorage fs;
//        Mat cameraMatrix1, cameraMatrix2, distCoeffs1, distCoeffs2, R1, P1, R2, P2, rmap11, rmap12, rmap21, rmap22;
//        fs.open("intrinsics.xml", FileStorage::READ);
//        fs["M1"] >> cameraMatrix1;
//        fs["M2"] >> cameraMatrix2;
//        fs["D1"] >> distCoeffs1;
//        fs["D2"] >> distCoeffs2;
//        fs.release();
//
//        fs.open("extrinsics.xml", FileStorage::READ);
//        fs["R1"] >> R1;
//        fs["R2"] >> R2;
//        fs["P1"] >> P1;
//        fs["P2"] >> P2;
//        fs.release();

        cv::initUndistortRectifyMap(K1, D1, R1, P1, frameSize, CV_32F, lmapx, lmapy);
        cv::initUndistortRectifyMap(K2, D2, R2, P2, frameSize, CV_32F, rmapx, rmapy);

//        auto calibrationData = *Camera::loadCalibrationDataFromXML("../config/stereo-left-640px.xml",
//                                                                    camLeft.getFrameSize());

        //create stereo BM
        const int ndisp = 64;
        const int blockSize = 19;
//        auto stereoMatcher = cuda::createStereoBM(ndisp, blockSize);
//        auto stereoMatcher = cuda::createStereoBeliefPropagation();
        auto stereoMatcher = cuda::createStereoConstantSpaceBP();

        //create disparity filter
//        auto stereoFilter = ximgproc::createDisparityWLSFilter(stereoMatcher);
//        stereoFilter->setLambda(8000.0);
//        stereoFilter->setSigmaColor(1.5);
        //auto stereoFilter = cuda::createDisparityBilateralFilter(); //TODO parameters

        while(true) {
            if (!camLeft.isOpen())
                throw runtime_error("Left camera not open");
            if (!camRight.isOpen())
                throw runtime_error("Right camera not open");

            //get frames
            camRight.grabFrame();
            camLeft.grabFrame();
            camRight.getGrabbedFrame(matRight);
            camLeft.getGrabbedFrame(matLeft);

            //undistort images
            remap(matLeft, matLeft, lmapx, lmapy, INTER_LINEAR);
            remap(matRight, matRight, rmapx, rmapy, INTER_LINEAR); //TODO use rmap2x

//            matLeft = camLeft.undistort(matLeft, calibrationData);
//            matRight = camRight.undistort(matRight, calibrationData);

            //upload images to GPU
            gpuMatLeft.upload(matLeft);
            gpuMatRight.upload(matRight);

            //on GPU: convert color
            cuda::cvtColor(gpuMatLeft, gpuMatLeft, COLOR_BGR2GRAY);
            cuda::cvtColor(gpuMatRight, gpuMatRight, COLOR_BGR2GRAY);

            //on GPU: generate LTR and RTL disparity maps
            stereoMatcher->compute(gpuMatLeft, gpuMatRight, gpuMatDisparityLeft);
            cuda::multiply(gpuMatDisparityLeft, Scalar(60.0), gpuMatDisparityLeft);

//            cuda::equalizeHist(gpuMatDisparityLeft, gpuMatDisparityLeft);

            gpuMatDisparityLeft.download(disparityLeft);

//            gpuMatDisparityRight.download(disparityRight);

            //Filter (on CPU)
//            stereoFilter->filter(disparityLeft, matLeft, disparityFiltered, disparityRight);

//            ximgproc::getDisparityVis(disparityLeft, disparityLeft, 20.0);

            disparityLeft.copyTo(disparityFiltered);

            ImageTransform::scale(matLeft, 4.0);
            ImageTransform::scale(matRight, 4.0);
            ImageTransform::scale(disparityFiltered, 4.0);

            cv::imshow("Left", matLeft);
            cv::imshow("Right", matRight);
            cv::imshow("Disparity", disparityFiltered);

//            cuda::cvtColor(gpuMatIn, gpuMatOut, COLOR_BGR2GRAY);
//            cuda::equalizeHist(gpuMatOut, gpuMatOut);
//            canny->detect(gpuMatOut, gpuMatOut);
//            cuda::cvtColor(gpuMatOut, gpuMatOut, COLOR_GRAY2BGR);
//            cuda::bitwise_not(gpuMatOut,gpuMatOut);
//            cuda::bitwise_and(gpuMatIn, gpuMatOut, gpuMatOut);
//            cuda::gammaCorrection(gpuMatOut, gpuMatOut);

//            cuda::threshold(gpuMatIn, gpuMatOut, 128.0, 255.0, THRESH_BINARY);

            cout << "frame " << std::setprecision(4) << camLeft.getPositionFrame() << " @ " << camLeft.getFrameRate() << " fps (" << camLeft.getPositionSeconds() << " sec)" << endl;

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