#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

using namespace cv;
using namespace std;

// Configuración del tablero de ajedrez
const Size chessboardSize(9, 6); // 10x7 cuadrados -> 9x6 esquinas internas
const float squareSize = 24.0f;  // Tamaño de cada cuadrado en mm

// Variables para la calibración
Mat cameraMatrix = (Mat_<double>(3, 3) << 1000, 0, 320, 0, 1000, 240, 0, 0, 1);
Mat distCoeffs = Mat::zeros(5, 1, CV_64F);

// Puntos 3D de la pirámide a proyectar (centrada en el tablero)
vector<Point3f> create3DPyramid(float baseSize = 96.0f, float height = -144.0f)
{
    vector<Point3f> pyramid;
    float halfSize = baseSize / 2.0f;
    
    // Calcular centro del tablero
    float centerX = (chessboardSize.width - 1) * squareSize / 2.0f;
    float centerY = (chessboardSize.height - 1) * squareSize / 2.0f;
    
    // Base de la pirámide (en el plano del tablero)
    pyramid.push_back(Point3f(centerX - halfSize, centerY - halfSize, 0)); // Punto 0: esquina inferior izquierda
    pyramid.push_back(Point3f(centerX + halfSize, centerY - halfSize, 0)); // Punto 1: esquina inferior derecha
    pyramid.push_back(Point3f(centerX + halfSize, centerY + halfSize, 0)); // Punto 2: esquina superior derecha
    pyramid.push_back(Point3f(centerX - halfSize, centerY + halfSize, 0)); // Punto 3: esquina superior izquierda
    
    // Vértice superior de la pirámide (centrado sobre la base)
    pyramid.push_back(Point3f(centerX, centerY, height)); // Punto 4: vértice
    
    return pyramid;
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

    namedWindow("OPTION 2", WINDOW_AUTOSIZE);

    // Generar puntos 3D del tablero
    vector<Point3f> chessboard3D;
    for (int i = 0; i < chessboardSize.height; i++)
    {
        for (int j = 0; j < chessboardSize.width; j++)
        {
            chessboard3D.push_back(Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    // Generar puntos 3D de la pirámide
    vector<Point3f> pyramid3D = create3DPyramid();

    // Buscar esquinas del tablero
    vector<Point2f> corners;
    bool found = findChessboardCorners(image, chessboardSize, corners, 
                                     CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);

    if (found)
    {
        // Refinar detección de esquinas
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                    TermCriteria(TermCriteria::EPS + TermCriteria::MAX_ITER, 30, 0.1));

        // Calcular pose de la cámara (parámetros extrínsecos)
        Mat rvec, tvec;
        solvePnP(chessboard3D, corners, cameraMatrix, distCoeffs, rvec, tvec);

        // Proyectar puntos 3D de la pirámide a 2D
        vector<Point2f> pyramid2D;
        projectPoints(pyramid3D, rvec, tvec, cameraMatrix, distCoeffs, pyramid2D);

        // Dibujar la pirámide
        // Base (cuadrado)
        for (int i = 0; i < 4; i++)
        {
            line(image, pyramid2D[i], pyramid2D[(i + 1) % 4], Scalar(0, 0, 255), 2);
        }
        
        // Lados (líneas del vértice a cada esquina de la base)
        for (int i = 0; i < 4; i++)
        {
            line(image, pyramid2D[i], pyramid2D[4], Scalar(0, 0, 255), 2);
        }

        // Mostrar ejes de coordenadas
        vector<Point3f> axisPoints = {
            Point3f(0, 0, 0),          // Origen
            Point3f(50, 0, 0),         // Eje X
            Point3f(0, 50, 0),         // Eje Y
            Point3f(0, 0, 50)          // Eje Z
        };
        
        vector<Point2f> axis2D;
        projectPoints(axisPoints, rvec, tvec, cameraMatrix, distCoeffs, axis2D);
        
    }
    else
    {
        putText(image, "Chessboard not found!", Point(20, 40), 
               FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);
    }

    imshow("OPTION 2", image);
    waitKey(0);

    return 0;
}
