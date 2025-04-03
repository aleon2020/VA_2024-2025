#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <fstream>

cv::Mat image, displayedImage, cameraMatrix, distCoeffs;
bool correctionApplied = false;
std::vector<std::vector<cv::Point2f>> chessboardPoints2D;
std::vector<std::vector<cv::Point3f>> chessboardPoints3D;

cv::Size chessboardSize(10, 7);
cv::Size squareSize(24, 24);

void detectChessboard(cv::Mat &img)
{
    std::vector<cv::Point2f> corners;
    bool found = cv::findChessboardCorners(img, chessboardSize, corners);
    if (found)
    {
        cv::drawChessboardCorners(img, chessboardSize, corners, found);
    }
}

void saveCalibration()
{
    cv::FileStorage file("calibration.yml", cv::FileStorage::WRITE);
    if (!file.isOpened())
    {
        std::cerr << "Error: No se pudo abrir el archivo para escritura." << std::endl;
        return;
    }

    file << "cameraMatrix" << cameraMatrix;
    file << "distCoeffs" << distCoeffs;
    file.release();
    std::cout << "Calibración guardada correctamente." << std::endl;
}


void loadCalibration()
{
    cv::FileStorage file("calibration.yml", cv::FileStorage::READ);
    if (!file.isOpened())
    {
        std::cerr << "Error: No se pudo abrir el archivo de calibración." << std::endl;
        return;
    }

    file["cameraMatrix"] >> cameraMatrix;
    file["distCoeffs"] >> distCoeffs;
    file.release();
    std::cout << "Calibración cargada correctamente." << std::endl;
}

void applyCorrection()
{
    if (!cameraMatrix.empty() && !distCoeffs.empty())
    {
        cv::Mat corrected;
        cv::undistort(image, corrected, cameraMatrix, distCoeffs);
        displayedImage = corrected;
    }
}

void trackbarCallback(int position, void *)
{
    if (position == 0)
    {
        displayedImage = image.clone();
    }
    else if (position == 1)
    {
        detectChessboard(displayedImage);
    }
    else if (position == 2)
    {
        // Aquí se implementaría solvePnP y projectPoints para proyectar un cubo o una figura más compleja
    }
    cv::imshow("Chessboard Calibration", displayedImage);
}

int main()
{
    image = cv::imread("chessboard.jpeg");
    if (image.empty())
    {
        std::cerr << "Error: No se pudo cargar la imagen." << std::endl;
        return -1;
    }

    displayedImage = image.clone();
    cv::namedWindow("Chessboard Calibration");
    int sliderValue = 0;
    cv::createTrackbar("Mode", "Chessboard Calibration", &sliderValue, 2, trackbarCallback);
    trackbarCallback(0, nullptr);

    while (true)
    {
        int key = cv::waitKey(30);
        if (key == 27) break; // ESC para salir
        if (sliderValue == 1)
        {
            if (key == 'c')
            {
                std::cout << "Calibrando..." << std::endl;
                // Implementar acumulación de puntos y calibración
            }
            else if (key == 's')
            {
                saveCalibration();
                std::cout << "Calibración guardada." << std::endl;
            }
            else if (key == 'a')
            {
                if (correctionApplied)
                {
                    displayedImage = image.clone();
                    correctionApplied = false;
                }
                else
                {
                    loadCalibration();
                    applyCorrection();
                    correctionApplied = true;
                }
            }
            cv::imshow("Chessboard Calibration", displayedImage);
        }
    }
    return 0;
}

