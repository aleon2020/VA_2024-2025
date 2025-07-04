#include <opencv2/highgui.hpp>

int main()
{
  // Create image variable
  // Las imágenes en OpenCV son tipo cv::Mat
  cv::Mat image;

  // Read image
  // Lectura de la imagen: Mat cv::imread(const string &filename, int flags=IMREAD_COLOR)
  image = cv::imread("../../data/lena.jpg", cv::IMREAD_COLOR);

  // Show image
  // Mostrar la imagen
  cv::imshow("TEST IMAGE", image);

  // Wait to press a key
  cv::waitKey(0);

  return 0;
}
