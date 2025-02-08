#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdio.h>
using namespace cv;
using namespace std;


int main(int argc, char ** argv)
{
  // Read image
  Mat src = imread("../../data/lena.jpg", 0);
  if (src.empty()) {
    cout << "the image is not exist" << endl;
    return -1;
  }

  resize(src, src, Size(512, 512));
  src.convertTo(src, CV_32F, 1.0 / 255);

  // Masks
  // Se crea la máscara con el tamaño deseado y se incializan 
  // los valores los cuales se multiplicarán por los valores de 
  // cada píxeles y se sumarán
  Mat M1 = (Mat_<char>(3, 3) << 1, 1, 1,
    1, 1, 1,
    1, 1, 1);

  Mat M2 = (Mat_<char>(3, 3) << 1, 2, 1,
    0, 0, 0,
    -1, -2, -1);


  Mat M3 = (Mat_<char>(3, 3) << 1, 0, -1,
    2, 0, -2,
    1, 0, -1);

  // Applying masks
  Mat dst1, dst2, dst3;

  // Se utiliza la función filter2D la cual se encarga de aplicar
  // la máscara a cada píxel teniendo en cuenta el tamaño de
  // vecindad elegido por la máscara
  filter2D(src, dst1, src.depth(), M1);
  filter2D(src, dst2, src.depth(), M2);
  filter2D(src, dst3, src.depth(), M3);

  // Show images
  imshow("Original", src);
  imshow("Mask 1", dst1);
  imshow("Mask 2", dst2);
  imshow("Mask 3", dst3);
  waitKey();

  return 0;
}
