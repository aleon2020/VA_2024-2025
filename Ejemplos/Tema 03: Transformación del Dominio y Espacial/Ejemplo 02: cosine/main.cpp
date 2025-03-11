#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
using namespace std;
using namespace cv;

int main()
{
  // Read image
  Mat src = imread("../../data/lena.jpg", 0);
  if (src.empty()) {
    cout << "the image is not exist" << endl;
    return -1;
  }

  // En este ejemplo se redimensiona la imagen
  resize(src, src, Size(512, 512));
  src.convertTo(src, CV_32F, 1.0 / 255);

  // Discrete Cosine Transform
  // Para crear la Transformada Discreta del Coseno (DCT en inglés)
  // se utiliza la función dct proporcionada por OpenCV
  Mat srcDCT;
  dct(src, srcDCT);

  // Inverse Discrete Cosine Transform
  // Para obtener la Transformada Discreta Inversa del Coseno
  // (IDCT en inglés) se utiliza la función idct proporcionada por OpenCV
  Mat InvDCT;
  idct(srcDCT, InvDCT);

  // Show images
  imshow("src", src);
  imshow("dct", srcDCT);
  imshow("idct", InvDCT);
  waitKey();

  return 0;
}
