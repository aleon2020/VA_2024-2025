#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char ** argv)
{
  // Load an image
  Mat src = imread("../../data/lena.jpg", IMREAD_COLOR);
  if (src.empty() ) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }

  // Create windows
  namedWindow("Original image", WINDOW_AUTOSIZE);
  namedWindow("Rotation", WINDOW_AUTOSIZE);

  imshow("Original image", src);

  // Rotation
  // Para rotar se necesitan:
  // 1. Punto en el que se va a rotar la imagen
  // 2. Ángulo de rotación: Un valor positivo rota en dirección contraria a las agujas del reloj.
  Mat rotation_dst;
  Point center = Point(src.cols / 2, src.rows / 2);
  double angle = -50.0;
  double scale = 0.6;
  // Creación de la matriz de rotación
  Mat rot_mat = getRotationMatrix2D(center, angle, scale);
  // A través de la función warpAffine se aplica la matriz de rotación a toda la imagen
  warpAffine(src, rotation_dst, rot_mat, src.size());
  imshow("Rotation", rotation_dst);

  waitKey();
  return 0;
}
