#include <opencv2/highgui.hpp>

int main()
{
  // Create image variable
  // Las imágenes en OpenCV son tipo cv::Mat
  cv::Mat image;

  // Read image
  // Lectura de la imagen: Mat cv::imread(const string &filename, int flags=IMREAD_COLOR)
  image = cv::imread("pinkballs.jpeg", cv::IMREAD_COLOR);

  // Show image
  // Mostrar la imagen
  cv::imshow("OPTION 5", image);

  // Wait to press a key
  cv::waitKey(0);

  return 0;
}
