#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

const cv::Size chessboardSize(9, 6);
const float squareSize = 24.0f;
std::vector<std::vector<cv::Point2f>> imagePoints;
std::vector<std::vector<cv::Point3f>> objectPoints;
cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
bool calibrated = false;
bool showUndistorted = false;

// generate3DChessboardCorners() function
// Generates the 3D coordinates of the chessboard points
std::vector<cv::Point3f> 
generate3DChessboardCorners()
{
    std::vector<cv::Point3f> corners;
    for (int i = 0; i < chessboardSize.height; i++) {
        for (int j = 0; j < chessboardSize.width; j++) {
            corners.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    return corners;
}

// performCalibration() function
// Performs camera calibration using the detected 2D and 3D points 
// generated, in addition to calculating the camera matrix and distortion 
// coefficients
void 
performCalibration(cv::Size imageSize)
{
    std::vector<cv::Mat> rvecs, tvecs;
    int flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_K3 | cv::CALIB_ZERO_TANGENT_DIST | cv::CALIB_FIX_PRINCIPAL_POINT;
    double reprojError = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs, flags);
    std::cout << "Reproyection error: " << reprojError << std::endl;
    std::cout << "Camera matrix:\n" << cameraMatrix << std::endl;
    std::cout << "Distortion coefficients:\n" << distCoeffs << std::endl;
    calibrated = true;
}

// saveCalibration() function
// Save the camera matrix and distortion coefficients to a YAML file
void 
saveCalibration(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::WRITE);
    if (fs.isOpened()) {
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();
        std::cout << "Calibration saved to " << filename << std::endl;
    } else {
        std::cerr << "Error: Could not open file to save calibration" << std::endl;
    }
}

// loadCalibration() function
// Load a previously saved calibration from a YAML file
bool 
loadCalibration(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (fs.isOpened()) {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();
        calibrated = true;
        std::cout << "Calibration loaded from " << filename << std::endl;
        return true;
    } else {
        std::cerr << "Error: Could not open file to load calibration" << std::endl;
        return false;
    }
}

// main() function
// Main function of the program
int 
main()
{
    cv::Mat image = cv::imread("chessboard.jpeg");
    if (image.empty()) {
        std::cerr << "Error: Could not load image chessboard.jpeg" << std::endl;
        return -1;
    }
    cv::Mat displayImage, undistortedImage;
    image.copyTo(displayImage);
    bool found = false;
    std::vector<cv::Point2f> corners;
    cv::namedWindow("OPTION 1", cv::WINDOW_AUTOSIZE);
    std::vector<cv::Point3f> chessboard3D = generate3DChessboardCorners();
    while (true) {
        image.copyTo(displayImage);
        found = findChessboardCorners(image, chessboardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
        if (found) {
            cv::Mat gray;
            cvtColor(image, gray, cv::COLOR_BGR2GRAY);
            cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(displayImage, chessboardSize, corners, found);
        }
        if (showUndistorted && calibrated) {
            undistort(displayImage, undistortedImage, cameraMatrix, distCoeffs);
            imshow("OPTION 1", undistortedImage);
        } else {
            imshow("OPTION 1", displayImage);
        }
        int key = cv::waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        } else if (key == 'c' && found) {
            imagePoints.push_back(corners);
            objectPoints.push_back(chessboard3D);
            performCalibration(image.size());
        } else if (key == 's') {
            saveCalibration("calibration.yml");
        } else if (key == 'a') {
            if (calibrated) {
                showUndistorted = !showUndistorted;
                std::cout << "Undistortion " << (showUndistorted ? "ON" : "OFF") << std::endl;
            } else {
                if (loadCalibration("calibration.yml")) {
                    showUndistorted = true;
                }
            }
        }
    }
    return 0;
}