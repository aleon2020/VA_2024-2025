#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char ** argv)
{
  // Load an image
  CommandLineParser parser(argc, argv, "{@input | lena.jpg | input image}");
  Mat src = imread(samples::findFile(parser.get<String>("@input") ) );
  if (src.empty() ) {
    cout << "Could not open or find the image!\n" << endl;
    cout << "Usage: " << argv[0] << " <Input image>" << endl;
    return -1;
  }

  // Points to the affine transform calculation
  // Se necesitan tres puntos en la imagen original y destino
  // para calcular la relación entre ellos y su transformada
  Point2f srcTri[3];
  srcTri[0] = Point2f(0.f, 0.f);
  srcTri[1] = Point2f(src.cols - 1.f, 0.f);
  srcTri[2] = Point2f(0.f, src.rows - 1.f);

  Point2f dstTri[3];
  dstTri[0] = Point2f(0.f, src.rows * 0.33f);
  dstTri[1] = Point2f(src.cols * 0.85f, src.rows * 0.25f);
  dstTri[2] = Point2f(src.cols * 0.15f, src.rows * 0.7f);

  // Calculate the affine transform
  // A través de la función getAffineTransform 
  // se calcula la transformada afín
  Mat warp_mat = getAffineTransform(srcTri, dstTri);
  Mat warp_dst = Mat::zeros(src.rows, src.cols, src.type() );

  // Apply the affine transform
  // A través de la función warpAffine 
  // se aplica la matriz con la transformada a toda la imagen
  warpAffine(src, warp_dst, warp_mat, warp_dst.size() );

  // Rotation point, angle and scale (optional)
  // Punto a partir del cual se va a rotar
  Point center = Point(warp_dst.cols / 2, warp_dst.rows / 2);
  // Ángulo de rotación (positivo en sentido antihorario)
  // y factor de escala (opcional)
  double angle = -50.0;
  double scale = 0.6;

  // Calculate and apply rotation matrix
  // Se genera la matriz de rotación
  Mat rot_mat = getRotationMatrix2D(center, angle, scale);

  // A través de la función warpAffine 
  // se aplica la matriz de rotación a toda la imagen
  Mat warp_rotate_dst;
  warpAffine(warp_dst, warp_rotate_dst, rot_mat, warp_dst.size() );

  // Show images
  imshow("Source image", src);
  imshow("Warp", warp_dst);
  imshow("Warp + Rotate", warp_rotate_dst);

  waitKey();
  return 0;
}
