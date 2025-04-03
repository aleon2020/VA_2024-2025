#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

const cv::Size chessboardSize(9, 6);
const float squareSize = 24.0f;
cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);

// create3DCube() function
// Generates the 3D coordinates of the 8 vertices of the cube
std::vector<cv::Point3f> 
create3DCube(float size = 96.0f, float height = -96.0f)
{
    std::vector<cv::Point3f> cube;
    float halfSize = size / 2.0f;
    float centerX = (chessboardSize.width - 1) * squareSize / 2.0f;
    float centerY = (chessboardSize.height - 1) * squareSize / 2.0f;
    cube.push_back(cv::Point3f(centerX - halfSize, centerY - halfSize, 0));
    cube.push_back(cv::Point3f(centerX + halfSize, centerY - halfSize, 0));
    cube.push_back(cv::Point3f(centerX + halfSize, centerY + halfSize, 0));
    cube.push_back(cv::Point3f(centerX - halfSize, centerY + halfSize, 0));
    cube.push_back(cv::Point3f(centerX - halfSize, centerY - halfSize, height));
    cube.push_back(cv::Point3f(centerX + halfSize, centerY - halfSize, height));
    cube.push_back(cv::Point3f(centerX + halfSize, centerY + halfSize, height));
    cube.push_back(cv::Point3f(centerX - halfSize, centerY + halfSize, height));
    return cube;
}

// main() function
// Main function of the program
int 
main()
{
    cv::Mat image = cv::imread("chessboard.jpeg");
    if (image.empty())
    {
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
    std::vector<cv::Point3f> cube3D = create3DCube();
    std::vector<cv::Point2f> corners;
    bool found = findChessboardCorners(image, chessboardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    if (found) {
        cv::Mat gray;
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        cv::Mat rvec, tvec;
        solvePnP(chessboard3D, corners, cameraMatrix, distCoeffs, rvec, tvec);
        std::vector<cv::Point2f> cube2D;
        projectPoints(cube3D, rvec, tvec, cameraMatrix, distCoeffs, cube2D);
        for (int i = 0; i < 4; i++) {
            cv::line(image, cube2D[i], cube2D[(i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
        for (int i = 0; i < 4; i++) {
            cv::line(image, cube2D[i], cube2D[i + 4], cv::Scalar(0, 0, 255), 2);
        }
        for (int i = 4; i < 8; i++) {
            cv::line(image, cube2D[i], cube2D[4 + (i + 1) % 4], cv::Scalar(0, 0, 255), 2);
        }
        std::vector<cv::Point3f> axisPoints = {
            cv::Point3f(0, 0, 0),
            cv::Point3f(50, 0, 0),
            cv::Point3f(0, 50, 0),
            cv::Point3f(0, 0, 50)
        };
        std::vector<cv::Point2f> axis2D;
        projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, axis2D);
    } else {
        cv::putText(image, "Chessboard not found!", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
    }
    cv::imshow("OPTION 2", image);
    cv::waitKey(0);
    return 0;
}
