#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

const cv::Size chessboardSize(9, 6);
const float squareSize = 24.0f;
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

// create3DCube() function
// Generates the 3D coordinates of the sphere
std::vector<cv::Point3f> 
create3DSphere(float radius = 72.0f, int segments = 30)
{
    std::vector<cv::Point3f> spherePoints;
    float centerX = (chessboardSize.width - 1) * squareSize / 2.0f;
    float centerY = (chessboardSize.height - 1) * squareSize / 2.0f;
    for (int i = 0; i <= segments; i++) {
        float lat = M_PI * (-0.5f + (float)i / segments);
        float y = radius * sin(lat) + centerY;
        float r = radius * cos(lat);
        for (int j = 0; j <= segments; j++) {
            float lng = 2 * M_PI * (float)j / segments;
            float x = r * cos(lng) + centerX;
            float z = r * sin(lng);
            spherePoints.push_back(cv::Point3f(x, y, z));
        }
    }
    return spherePoints;
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
    cv::namedWindow("OPTION 2", cv::WINDOW_AUTOSIZE);
    std::vector<cv::Point3f> chessboard3D;
    for (int i = 0; i < chessboardSize.height; i++) {
        for (int j = 0; j < chessboardSize.width; j++) {
            chessboard3D.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    std::vector<cv::Point3f> sphere3D = create3DSphere();
    std::vector<cv::Point2f> corners;
    bool found = findChessboardCorners(image, chessboardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
        cv::Mat gray;
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        cv::Mat rvec, tvec;
        cv::solvePnP(chessboard3D, corners, cameraMatrix, distCoeffs, rvec, tvec);
        std::vector<cv::Point2f> sphere2D;
        cv::projectPoints(sphere3D, rvec, tvec, cameraMatrix, distCoeffs, sphere2D);
        int segments = 30;
        for (int i = 0; i < segments; i++) {
            for (int j = 0; j < segments; j++) {
                int idx1 = i * (segments + 1) + j;
                int idx2 = idx1 + 1;
                int idx3 = (i + 1) * (segments + 1) + j;
                int idx4 = idx3 + 1;
                cv::line(image, sphere2D[idx1], sphere2D[idx2], cv::Scalar(0, 0, 255), 1);
                cv::line(image, sphere2D[idx1], sphere2D[idx3], cv::Scalar(0, 0, 255), 1);
            }
        }
    } else {
        cv::putText(image, "Chessboard not found!", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("OPTION 2", image);
    cv::waitKey(0);
    return 0;
}