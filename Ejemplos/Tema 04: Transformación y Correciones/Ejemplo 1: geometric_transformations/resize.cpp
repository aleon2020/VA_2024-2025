#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char ** argv)
{
  // Load an image
  Mat src = imread("../../data/cat-small.jpg", IMREAD_COLOR);
  if (src.empty() ) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }

  // Create windows
  namedWindow("Original image", WINDOW_AUTOSIZE);
  namedWindow("Resize x10", WINDOW_AUTOSIZE);
  namedWindow("Resize /5", WINDOW_AUTOSIZE);

  imshow("Original image", src);

  // Resize
  // INTER_NEAREST. Es una interpolación del vecino cercano
  // INTER_LINEAR. Interpolación bilineal
  // INTER_CUBIC. Interpolación bicúbica (área de 4x4 píxeles)
  Mat resize_dst;
  int size_col = 10, size_row = 10;
  // Aumento de una imagen utilizando la interpolación por vecinos cercanos
  resize(src, resize_dst, cv::Size(), size_col, size_row, INTER_NEAREST);
  imshow("Resize x10", resize_dst);

  // Resampling
  Mat resize_dst2;
  int res = 5;
  // Reducción de una imagen utilizando la interpolación bicúbica
  resize(resize_dst, resize_dst2, Size(resize_dst.cols / res, resize_dst.rows / res), INTER_CUBIC);
  imshow("Resize /5", resize_dst2);

  waitKey();
  return 0;
}
