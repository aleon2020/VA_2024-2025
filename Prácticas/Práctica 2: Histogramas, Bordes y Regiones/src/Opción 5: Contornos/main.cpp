#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

using namespace cv;
using namespace std;

// Variables globales
Mat image, gray, edges, gauss;
int areaThreshold = 1000; // Valor inicial del umbral

// Callback para el trackbar
void onTrackbar(int, void*) {}

int main()
{
    // Leer imagen
    image = imread("contours.jpg", IMREAD_COLOR);
    if (image.empty()) {
        cout << "No se pudo cargar la imagen!" << endl;
        return -1;
    }

    // Convertir a escala de grises y aplicar desenfoque gaussiano
    cvtColor(image, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gauss, Size(5, 5), 0);

    // Crear una ventana con trackbar
    namedWindow("Detected Rectangles", WINDOW_AUTOSIZE);
    createTrackbar("Area Threshold", "Detected Rectangles", &areaThreshold, 20000, onTrackbar);

    while (true) {
        // Detección de bordes
        Canny(gauss, edges, 50, 100, 3);

        // Encontrar contornos
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(edges, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        int rectangleCount = 0;

        // Dibujar contornos en la imagen original
        Mat result = image.clone();
        for (size_t i = 0; i < contours.size(); i++) {
            double area = contourArea(contours[i]);
            if (area > areaThreshold) {
                RotatedRect rect = minAreaRect(contours[i]);
                Point2f vertices[4];
                rect.points(vertices);

                // Verificar si el contorno tiene cuatro lados mediante el rectángulo delimitador
                if (contours[i].size() >= 4) {
                    for (int j = 0; j < 4; j++) {
                        line(result, vertices[j], vertices[(j + 1) % 4], Scalar(0, 0, 255), 2);
                    }
                    rectangleCount++;

                    // Calcular centroide
                    Moments M = moments(contours[i]);
                    if (M.m00 != 0) {
                        int Cx = int(M.m10 / M.m00);
                        int Cy = int(M.m01 / M.m00);
                        circle(result, Point(Cx, Cy), 4, Scalar(255, 0, 0), -1);
                    }
                }
            }
        }

        // Mostrar número de rectángulos detectados
        putText(result, "Rectangles: " + to_string(rectangleCount), Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 2);
        imshow("Detected Rectangles", result);

        // Salir con la tecla ESC
        if (waitKey(30) == 27) break;
    }

    return 0;
}

