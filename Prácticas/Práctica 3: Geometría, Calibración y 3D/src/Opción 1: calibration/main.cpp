#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

using namespace cv;
using namespace std;

// Configuración del tablero de ajedrez
const Size chessboardSize(9, 6); // 10x7 cuadrados -> 9x6 esquinas internas
const float squareSize = 24.0f;   // Tamaño de cada cuadrado en mm

// Variables para la calibración
vector<vector<Point2f>> imagePoints;
vector<vector<Point3f>> objectPoints;
Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
bool calibrated = false;
bool showUndistorted = false;

// Generar puntos 3D del tablero
vector<Point3f> generate3DChessboardCorners()
{
    vector<Point3f> corners;
    for (int i = 0; i < chessboardSize.height; i++)
    {
        for (int j = 0; j < chessboardSize.width; j++)
        {
            corners.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }
    return corners;
}

// Calibrar la cámara
void performCalibration(Size imageSize)
{
    vector<Mat> rvecs, tvecs;
    int flags = CALIB_FIX_ASPECT_RATIO | CALIB_FIX_K3 | 
                CALIB_ZERO_TANGENT_DIST | CALIB_FIX_PRINCIPAL_POINT;
    
    double reprojError = calibrateCamera(objectPoints, imagePoints, imageSize, 
                                      cameraMatrix, distCoeffs, rvecs, tvecs, flags);
    
    cout << "Reproyection error: " << reprojError << endl;
    cout << "Camera matrix:\n" << cameraMatrix << endl;
    cout << "Distortion coefficients:\n" << distCoeffs << endl;
    
    calibrated = true;
}

// Guardar calibración en archivo
void saveCalibration(const string& filename)
{
    FileStorage fs(filename, FileStorage::WRITE);
    if (fs.isOpened())
    {
        fs << "camera_matrix" << cameraMatrix;
        fs << "distortion_coefficients" << distCoeffs;
        fs.release();
        cout << "Calibration saved to " << filename << endl;
    }
    else
    {
        cerr << "Error: Could not open file to save calibration" << endl;
    }
}

// Cargar calibración desde archivo
bool loadCalibration(const string& filename)
{
    FileStorage fs(filename, FileStorage::READ);
    if (fs.isOpened())
    {
        fs["camera_matrix"] >> cameraMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();
        calibrated = true;
        cout << "Calibration loaded from " << filename << endl;
        return true;
    }
    else
    {
        cerr << "Error: Could not open file to load calibration" << endl;
        return false;
    }
}

int main()
{
    // Cargar imagen
    Mat image = imread("chessboard.jpeg");
    if (image.empty())
    {
        cerr << "Error: Could not load image chessboard.jpeg" << endl;
        return -1;
    }

    Mat displayImage, undistortedImage;
    image.copyTo(displayImage);
    bool found = false;
    vector<Point2f> corners;

    namedWindow("OPTION 1", WINDOW_AUTOSIZE);

    // Generar puntos 3D del tablero (solo una vez)
    vector<Point3f> chessboard3D = generate3DChessboardCorners();

    while (true)
    {
        image.copyTo(displayImage);

        // Buscar esquinas del tablero
        found = findChessboardCorners(image, chessboardSize, corners, 
                                    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

        // Si se encontraron esquinas, refinarlas y dibujarlas
        if (found)
        {
            Mat gray;
            cvtColor(image, gray, COLOR_BGR2GRAY);
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), 
                        TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));
            
            drawChessboardCorners(displayImage, chessboardSize, corners, found);
        }

        // Mostrar imagen original o corregida
        if (showUndistorted && calibrated)
        {
            undistort(displayImage, undistortedImage, cameraMatrix, distCoeffs);
            imshow("OPTION 1", undistortedImage);
        }
        else
        {
            imshow("OPTION 1", displayImage);
        }

        // Manejo de teclas
        int key = waitKey(30);

        if (key == 'q' || key == 27) // 'q' o ESC para salir
        {
            break;
        }
        else if (key == 'c' && found) // 'c' para calibrar
        {
            imagePoints.push_back(corners);
            objectPoints.push_back(chessboard3D);
            performCalibration(image.size());
        }
        else if (key == 's') // 's' para guardar calibración
        {
            saveCalibration("calibration.yml");
        }
        else if (key == 'a') // 'a' para alternar corrección
        {
            if (calibrated)
            {
                showUndistorted = !showUndistorted;
                cout << "Undistortion " << (showUndistorted ? "ON" : "OFF") << endl;
            }
            else
            {
                if (loadCalibration("calibration.yml"))
                {
                    showUndistorted = true;
                }
            }
        }
    }

    return 0;
}
