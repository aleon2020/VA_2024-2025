#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>
#include <math.h>

// Igual que en el ejemplo anterior, pero usando el namespace
// evitamos tener que utilizar elementos de OpenCV con cv:: y std::
using namespace cv;
using namespace std;

int main(int argc, char ** argv)
{
  // Load an image
  // Lectura de la imagen
  Mat src = imread("../../data/RGB.jpg", IMREAD_COLOR);
  if (src.empty() ) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }

  // Split BGR channels
  vector<Mat> BGR_channels;
  split(src, BGR_channels);

  // Resize images and concatenate them
  int tam = 2;
  int h = src.size().height / tam, w = src.size().width / tam;

  cv::resize(src, src, Size(h, w), 0, 0, INTER_LANCZOS4);
  cv::resize(BGR_channels[0], BGR_channels[0], Size(w, h), 0, 0, INTER_LANCZOS4);
  cv::resize(BGR_channels[1], BGR_channels[1], Size(w, h), 0, 0, INTER_LANCZOS4);
  cv::resize(BGR_channels[2], BGR_channels[2], Size(w, h), 0, 0, INTER_LANCZOS4);

  cv::Mat win_mat1_rgb(cv::Size(BGR_channels[0].size().height, BGR_channels[0].size().width * 2),
    CV_8UC3);
  cv::Mat win_mat2_rgb(cv::Size(
      win_mat1_rgb.size().height,
      (win_mat1_rgb.size().width + BGR_channels[1].size().width)), CV_8UC3);
  cv::hconcat(BGR_channels[0], BGR_channels[1], win_mat1_rgb);
  cv::hconcat(win_mat1_rgb, BGR_channels[2], win_mat2_rgb);

  // Show image
  // Mostrar la imagen
  namedWindow("BGR Original", WINDOW_AUTOSIZE);
  imshow("BGR Original", src);
  cv::imshow("BGR Channels", win_mat2_rgb);

  // Changing original image to HSV using cvtColor
  // Para realizar un cambio de espacio de color, se utiliza la
  // función cvtColor(im_origen, im_destino, formato)
  Mat HSV_opencv;
  cvtColor(src, HSV_opencv, COLOR_BGR2HSV);
  imshow("HSV OpenCV", HSV_opencv);

  waitKey(0);
  return 0;
}
