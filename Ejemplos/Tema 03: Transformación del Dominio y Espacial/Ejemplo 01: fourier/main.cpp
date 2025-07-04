#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include <iostream>

using namespace cv;
using namespace std;

static void help(char ** argv)
{
  cout << endl
       << "This program demonstrated the use of the discrete Fourier transform (DFT). " << endl
       << "The dft of an image is taken and it's power spectrum is displayed." << endl << endl
       << "Usage:" << endl
       << argv[0] << " [image_name -- default images/lena.jpg]" << endl << endl;
}

// Compute the Discrete Fourier Transform
Mat computeDFT(const Mat & image)
{
  // Expand the image to an optimal size - power-of-two.
  Mat padded;
  // Este ajuste en el tamaño es para mejorar el rendimiento del algoritmo
  int m = getOptimalDFTSize(image.rows);
  int n = getOptimalDFTSize(image.cols);     // on the border add zero values
  copyMakeBorder(
    image, padded, 0, m - image.rows, 0, n - image.cols, BORDER_CONSTANT, Scalar::all(0));

  // Create a matrix for the real part of the image by converting the padded image to float
  Mat realPart = Mat_<float>(padded);

  // Create a matrix for the imaginary part of the complex values filled with zeros
  Mat imaginaryPart = Mat::zeros(padded.size(), CV_32F);

  // Combine the real and imaginary parts into a single complex matrix
  // El resultado de la Transformada de Fourier es complejo
  Mat planes[] = {realPart, imaginaryPart};
  Mat complexI;
  merge(planes, 2, complexI); // The resulting complex matrix has real and imaginary parts

  // Make the Discrete Fourier Transform
  // Se calcula la Transformada de Fourier Discreta (DFT en inglés) a través de DFT
  dft(complexI, complexI, DFT_COMPLEX_OUTPUT);  // this way the result may fit in the source matrix
  return complexI;
}

// Crop and rearrange
Mat fftShift(const Mat & magI)
{
  Mat magI_copy = magI.clone();
  // crop the spectrum, if it has an odd number of rows or columns
  magI_copy = magI_copy(Rect(0, 0, magI_copy.cols & -2, magI_copy.rows & -2));

  // rearrange the quadrants of Fourier image  so that the origin is at the image center
  int cx = magI_copy.cols / 2;
  int cy = magI_copy.rows / 2;

  // Reordena los cuadrantes
  Mat q0(magI_copy, Rect(0, 0, cx, cy));     // Top-Left - Create a ROI per quadrant
  Mat q1(magI_copy, Rect(cx, 0, cx, cy));    // Top-Right
  Mat q2(magI_copy, Rect(0, cy, cx, cy));    // Bottom-Left
  Mat q3(magI_copy, Rect(cx, cy, cx, cy));   // Bottom-Right

  Mat tmp;                                   // swap quadrants (Top-Left with Bottom-Right)
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);

  q1.copyTo(tmp);                            // swap quadrant (Top-Right with Bottom-Left)
  q2.copyTo(q1);
  tmp.copyTo(q2);

  return magI_copy;
}


// Calculate dft spectrum
Mat spectrum(const Mat & complexI)
{
  Mat complexImg = complexI.clone();
  // Shift quadrants
  // Se reordenan los cuadrantes
  Mat shift_complex = fftShift(complexImg);

  // Transform the real and complex values to magnitude
  // compute the magnitude and switch to logarithmic scale
  // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
  // Un número complejo tiene una parte real (Re) y una compleja (imaginaria - Im).
  // Los resultados de una DFT son números complejos. La magnitud de una DFT es:
  // M = [[Re(DFT(I))^2]+[Im(DFT(I))^2]]^1/2
  Mat planes_spectrum[2];
  split(shift_complex, planes_spectrum);         
  // planes_spectrum[0] = Re(DFT(I)), planes_spectrum[1] = Im(DFT(I))
  magnitude(planes_spectrum[0], planes_spectrum[1], planes_spectrum[0]);  
  // planes_spectrum[0] = magnitude
  Mat spectrum = planes_spectrum[0];

  // Switch to a logarithmic scale
  spectrum += Scalar::all(1);
  log(spectrum, spectrum);

  // Normalize
  normalize(spectrum, spectrum, 0, 1, NORM_MINMAX);   // Transform the matrix with float values into a
                                                      // viewable image form (float between values 0 and 1)
  return spectrum;
}

int main(int argc, char ** argv)
{
  help(argv);
  const char * filename = argc >= 2 ? argv[1] : "lena.jpg";
  Mat I = imread(samples::findFile(filename), IMREAD_GRAYSCALE);
  if (I.empty()) {
    cout << "Error opening image" << endl;
    return EXIT_FAILURE;
  }

  // Read image I
  // Compute the Discrete fourier transform
  // Se calcula la transformada de Fourier
  Mat complexImg = computeDFT(I);

  // Get the spectrum
  // Se obtiene el espectro de la DFT
  Mat spectrum_original = spectrum(complexImg);

  // Crop and rearrange
  // Se reordenan los cuadrantes para poner las bajas
  // frecuencias en el centro antes de procesar la DFT
  Mat shift_complex = fftShift(complexImg);   // Rearrange quadrants - Spectrum with low 
                                              // values at center - Theory mode

  // doSomethingWithTheSpectrum(shift_complex);
  // Se reordenan los cuadrantes para poner las bajas frecuencias 
  // en las esquinas que es como OpenCV lo procesa
  Mat rearrange = fftShift(shift_complex);    // Rearrange quadrants - Spectrum with low 
                                              // values at corners - OpenCV mode

  // Get the spectrum after the processing
  Mat spectrum_filter = spectrum(rearrange);

  // Original image
  imshow("Input Image", I);
  // Show the spectrums
  imshow("Spectrum original", spectrum_original);
  imshow("Spectrum filter", spectrum_filter);

  // Calculating the idft
  // Para calcular la inversa de la transformada, utilizamos la función idft 
  // aplicada a los valores complejos obtenidos con computeDFT
  Mat inverseTransform;
  idft(rearrange, inverseTransform, cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
  normalize(inverseTransform, inverseTransform, 0, 1, NORM_MINMAX);
  imshow("Reconstructed", inverseTransform);

  waitKey(0);
  return EXIT_SUCCESS;
}
