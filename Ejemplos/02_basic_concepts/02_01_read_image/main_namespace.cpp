#include <opencv2/highgui.hpp>

// Igual que en el ejemplo anterior, pero usando el namespace
// evitamos tener que utilizar elementos de OpenCV con cv::
using namespace cv;

int main()
{
  // Create image variable
  // Las imágenes en OpenCV son tipo cv::Mat
  Mat image;

  // Read image
  // Lectura de la imagen: Mat cv::imread(const string &filename, int flags=IMREAD_COLOR)
  image = imread("../../data/lena.jpg", IMREAD_COLOR);

  // Show image
  // Mostrar la imagen
  imshow("TEST IMAGE", image);

  // Wait to press a key
  waitKey(0);

  return 0;
}
