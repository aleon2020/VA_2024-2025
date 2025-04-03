#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>

// main() function
// Main function of the program
int 
main() 
{
    cv::Mat left = cv::imread("pinkballs_1.jpeg", cv::IMREAD_GRAYSCALE);
    cv::Mat right = cv::imread("pinkballs_2.jpeg", cv::IMREAD_GRAYSCALE);
    if (left.empty() || right.empty()) {
        std::cerr << "Error: No se pudieron cargar las imágenes estéreo rectificadas" << std::endl;
        return -1;
    }
    int numDisparities = 64;
    int blockSize = 15;
    cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(numDisparities, blockSize);
    stereo->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
    stereo->setPreFilterSize(9);
    stereo->setPreFilterCap(31);
    stereo->setTextureThreshold(10);
    stereo->setUniquenessRatio(15);
    stereo->setSpeckleWindowSize(100);
    stereo->setSpeckleRange(32);
    stereo->setDisp12MaxDiff(1);
    cv::Mat disparity, disparityNormalized;
    stereo->compute(left, right, disparity);
    cv::normalize(disparity, disparityNormalized, 0, 255, cv::NORM_MINMAX, CV_8U);
    cv::namedWindow("LEFT IMAGE", cv::WINDOW_NORMAL);
    cv::imshow("LEFT IMAGE", left);
    cv::namedWindow("RIGHT_IMAGE", cv::WINDOW_NORMAL);
    cv::imshow("RIGHT_IMAGE", right);
    cv::namedWindow("OPTION 3", cv::WINDOW_NORMAL);
    cv::imshow("OPTION 3", disparityNormalized);
    cv::waitKey(0);
    return 0;
}