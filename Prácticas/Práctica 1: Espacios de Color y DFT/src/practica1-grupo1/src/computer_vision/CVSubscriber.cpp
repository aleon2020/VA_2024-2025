#include "computer_vision/CVSubscriber.hpp"

// FUNCTIONS FOR CALCULATION OF THE DISCRETE FOURIER TRANSFORM (DFT)

// computeDFT() function
// Compute the Discrete fourier transform
cv::Mat computeDFT(const cv::Mat & image)
{
  // Expand the image to an optimal size.
  cv::Mat padded;

  // This size adjustment is to improve the performance of the algorithm
  int m = cv::getOptimalDFTSize(image.rows);
  int n = cv::getOptimalDFTSize(image.cols);
  cv::copyMakeBorder(
    image, padded, 0, m - image.rows, 0, n - image.cols, cv::BORDER_CONSTANT, cv::Scalar::all(0));

  // Combine the real and imaginary parts into a single complex matrix
  // (the result of the Fourier Transform is complex)
  cv::Mat planes[] = {cv::Mat_<float>(padded), cv::Mat::zeros(padded.size(), CV_32F)};
  cv::Mat complexI;
  cv::merge(planes, 2, complexI); // The resulting complex matrix has real and imaginary parts

  // Make the Discrete Fourier Transform
  // The Discrete Fourier Transform (DFT) is calculated through DFT
  cv::dft(complexI, complexI, cv::DFT_COMPLEX_OUTPUT); // This way the result may fit in the source matrix
  return complexI;
}

// fftShift() function
// Crop and rearrange
cv::Mat fftShift(const cv::Mat & magI)
{
  cv::Mat magI_copy = magI.clone();

  // Crop the spectrum, if it has an odd number of rows or columns
  magI_copy = magI_copy(cv::Rect(0, 0, magI_copy.cols & -2, magI_copy.rows & -2));

  // Rearrange the quadrants of Fourier image so that the origin is at the image center
  int cx = magI_copy.cols / 2;
  int cy = magI_copy.rows / 2;

  // Rearrange the quadrants
  cv::Mat q0(magI_copy, cv::Rect(0, 0, cx, cy));     // Top-Left - Create a ROI per quadrant
  cv::Mat q1(magI_copy, cv::Rect(cx, 0, cx, cy));    // Top-Right
  cv::Mat q2(magI_copy, cv::Rect(0, cy, cx, cy));    // Bottom-Left
  cv::Mat q3(magI_copy, cv::Rect(cx, cy, cx, cy));   // Bottom-Right

  cv::Mat tmp;                                       // swap quadrants (Top-Left with Bottom-Right)
  q0.copyTo(tmp);
  q3.copyTo(q0);
  tmp.copyTo(q3);

  q1.copyTo(tmp);                                    // swap quadrant (Top-Right with Bottom-Left)
  q2.copyTo(q1);
  tmp.copyTo(q2);

  return magI_copy;
}

// spectrum() function
// Calculate dft spectrum
cv::Mat spectrum(const cv::Mat & complexI)
{
  cv::Mat complexImg = complexI.clone();

  // Shift quadrants
  // The quadrants are rearranged
  cv::Mat shift_complex = fftShift(complexImg);

  // Transform the real and complex values to magnitude
  // compute the magnitude and switch to logarithmic scale
  // => log(1 + sqrt(Re(DFT(I))^2 + Im(DFT(I))^2))
  // A complex number has a real part (Re) and a complex part (imaginary - Im)
  // The results of a DFT are complex numbers. The magnitude of a DFT is:
  // M = [[Re(DFT(I))^2] + [Im(DFT(I))^2]] ^ 1/2
  cv::Mat planes_spectrum[2];
  cv::split(shift_complex, planes_spectrum);

  // planes_spectrum[0] = Re(DFT(I), planes_spectrum[1] = Im(DFT(I))
  cv::magnitude(planes_spectrum[0], planes_spectrum[1], planes_spectrum[0]);

  // planes_spectrum[0] = magnitude
  cv::Mat spectrum = planes_spectrum[0];

  // Switch to a logarithmic scale
  spectrum += cv::Scalar::all(1);
  cv::log(spectrum, spectrum);

  // Normalize
  cv::normalize(spectrum, spectrum, 0, 1, cv::NORM_MINMAX);   // Transform the matrix with float values into a
  return spectrum;                                            // viewable image form (float between values 0 and 1).
}

// GLOBAL PARAMETERS (NAMESPACE)

namespace CVParams
{

// Execution control
bool running = false;

// Defining the name of the window and trackbars
inline std::string WINDOW_NAME = "Practice 1";
inline std::string OPTION = "Option [0-5]";
inline std::string COLOR_SELECTION = "Color selection [0-3]";
inline std::string DFT_FILTER_SIZE = "DFT Value [0-100]";

// filterMode() enum
// Enumeration of frequency filter modes
typedef enum _filterMode
{
  LOW_PASS_FILTER = 0,
  HIGH_PASS_FILTER
} filterMode;

// colorSpace() enum
// Enumeration of color spaces
typedef enum _colorSpace
{
  BGR = 0,
  HSV
} colorSpace;

}

// COLOR SPACES (NAMESPACE)

namespace CVColors
{

// Pink color
inline std::vector<cv::Scalar> HSV_PINK = {cv::Scalar(140, 50, 25), cv::Scalar(170, 255, 255)};

// Red color
inline std::vector<cv::Scalar> BGR_RED = {cv::Scalar(0, 0, 60), cv::Scalar(50, 25, 150)};

// Green color
inline std::vector<cv::Scalar> BGR_GREEN = {cv::Scalar(0, 85, 0), cv::Scalar(80, 255, 80)};

// Blue color
inline std::vector<cv::Scalar> BGR_BLUE = {cv::Scalar(100, 0, 0), cv::Scalar(255, 130, 60)};

// Yellow color
inline std::vector<cv::Scalar> BGR_YELLOW = {cv::Scalar(0, 80, 150), cv::Scalar(80, 255, 255)};

}

// IMAGE PROCESSING FUNCTIONS (NAMESPACE)

namespace CVFunctions
{

// bgr_to_hsv() function
// Convert a BGR image to HSV
inline cv::Mat bgr_to_hsv(const cv::Mat & bgr_img)
{
  cv::Mat hsv_img;
  cv::cvtColor(bgr_img, hsv_img, cv::COLOR_BGR2HSV);
  return hsv_img;
}

// mask_from_range() function
// Create a binary mask of an image based on a range of colors
cv::Mat mask_from_range(
  const cv::Mat & img, const cv::Scalar & lower_bound,
  const cv::Scalar & upper_bound)
{
  cv::Mat mask;
  cv::inRange(img, lower_bound, upper_bound, mask);
  return mask;
}

// color_filter() function
// Filters an image depending on the color range in a specific color space
cv::Mat color_filter(
  const cv::Mat & img, const CVParams::colorSpace color_space,
  const std::vector<cv::Scalar> & color_range)
{
  cv::Mat mask;
  cv::Mat filtered_img;
  cv::Mat preprocessed_img;

  if (color_space == CVParams::HSV) {
    cv::cvtColor(img, preprocessed_img, cv::COLOR_BGR2HSV);
  } else {
    preprocessed_img = img;
  }

  mask = mask_from_range(preprocessed_img, color_range[0], color_range[1]);
  cv::copyTo(img, filtered_img, mask);
  return filtered_img;
}

// create_dft_filter() function
// Create a low / high pass filter for the Fourier Transform
cv::Mat create_dft_filter(
  const cv::Mat & complexI, const int dft_filter_size,
  const CVParams::filterMode mode)
{
  cv::Mat filter = cv::Mat::zeros(complexI.size(), CV_MAT_DEPTH(complexI.type()));

  cv::Mat planes[] = {filter, cv::Mat::zeros(filter.size(), CV_MAT_DEPTH(complexI.type()))};
  cv::Mat filter_complex;

  int cx = complexI.cols / 2;
  int cy = complexI.rows / 2;

  cv::circle(filter, cv::Point(cx, cy), dft_filter_size, cv::Scalar(1.0), -1);

  if (mode == CVParams::HIGH_PASS_FILTER) {
    filter = cv::Mat::ones(filter.size(), filter.type()) - filter;
  }

  cv::merge(planes, 2, filter_complex);
  return filter_complex;
}

// dft_filter() function
// Applies a low / high pass filter for the Fourier Transform
cv::Mat dft_filter(
  const cv::Mat & og_img, const int dft_filter_size,
  const CVParams::filterMode mode)
{
  cv::Mat gray_img;
  cv::Mat complex_img;
  cv::Mat processed_img;
  cv::Mat inverse_transformed_img;
  cv::Mat filtered_complex_img;

  cv::cvtColor(og_img, gray_img, cv::COLOR_BGR2GRAY);
  complex_img = computeDFT(gray_img);
  cv::Mat filter = create_dft_filter(complex_img, dft_filter_size, mode);
  cv::mulSpectrums(fftShift(complex_img), filter, filtered_complex_img, 0);
  cv::idft(fftShift(filtered_complex_img), inverse_transformed_img,
      cv::DFT_INVERSE | cv::DFT_REAL_OUTPUT);
  cv::normalize(inverse_transformed_img, processed_img, 0, 1, cv::NORM_MINMAX);
  return processed_img;
}

}

// init_window() function
// Initialize the window and trackbars
void initWindow()
{
  if (CVParams::running) {
    return;
  }

  CVParams::running = true;
  cv::namedWindow(CVParams::WINDOW_NAME);
  cv::createTrackbar(CVParams::OPTION, CVParams::WINDOW_NAME, nullptr, 5, 0);
  cv::createTrackbar(CVParams::COLOR_SELECTION, CVParams::WINDOW_NAME, nullptr, 3, 0);
  cv::createTrackbar(CVParams::DFT_FILTER_SIZE, CVParams::WINDOW_NAME, nullptr, 100, 0);
  cv::setTrackbarPos(CVParams::OPTION, CVParams::WINDOW_NAME, 0);
  cv::setTrackbarPos(CVParams::COLOR_SELECTION, CVParams::WINDOW_NAME, 0);
  cv::setTrackbarPos(CVParams::DFT_FILTER_SIZE, CVParams::WINDOW_NAME, 50);
}

// MAIN PROGRAM (NAMESPACE)

namespace computer_vision
{

CVGroup CVSubscriber::processing(
  const cv::Mat rgb,
  const cv::Mat depth,
  const cv::Mat disparity,
  const cv::Mat left_rect,
  const cv::Mat right_rect,
  const cv::Mat left_raw,
  const cv::Mat right_raw,
  const pcl::PointCloud<pcl::PointXYZRGB> in_pointcloud)
const
{
  cv::Mat og_img = rgb.clone();
  cv::Mat processed_img;
  std::vector<cv::Scalar> color_range;
  int option, color_selection, dft_filter_size;

  if (og_img.empty()) {
    return CVGroup(
      rgb,
      depth,
      disparity,
      left_rect,
      right_rect,
      left_raw,
      right_raw,
      in_pointcloud
    );
  }

  initWindow();

  option = cv::getTrackbarPos(CVParams::OPTION, CVParams::WINDOW_NAME);
  color_selection = cv::getTrackbarPos(CVParams::COLOR_SELECTION, CVParams::WINDOW_NAME);
  dft_filter_size = cv::getTrackbarPos(CVParams::DFT_FILTER_SIZE, CVParams::WINDOW_NAME);

  switch(option) {

    // OPTION 0
    // Display the image in BGR color format
    case 0:
      processed_img = og_img;
      break;

    // OPTION 1
    // Convert the image to HSV color space using
    // the cvtColor function and display the result
    case 1:
      processed_img = CVFunctions::bgr_to_hsv(og_img);
      break;

    // OPTION 2
    // Show only the pink ball by applying a color filter
    // about HSV space
    case 2:
      processed_img = CVFunctions::color_filter(og_img, CVParams::HSV, CVColors::HSV_PINK);
      break;

    // OPTION 3
    // Depending on the value of the second slider, only one will be shown
    // of the cubes (0 – red, 1 – green, 2 – blue, 3 – yellow). For this, they will apply
    // the corresponding color filters in the BGR space
    case 3:
      switch (color_selection) {
        case 0:
          color_range = CVColors::BGR_RED;
          break;
        case 1:
          color_range = CVColors::BGR_GREEN;
          break;
        case 2:
          color_range = CVColors::BGR_BLUE;
          break;
        case 3:
          color_range = CVColors::BGR_YELLOW;
          break;
        default:
          color_range = CVColors::BGR_RED;
          break;
      }

      processed_img = CVFunctions::color_filter(og_img, CVParams::BGR, color_range);
      break;

    // OPTION 4
    // Apply a low pass filter to the Fourier spectrum and display the image
    // obtained after the inverse transform. The filter size will depend
    // of the value selected in the third slider
    case 4:
      processed_img = CVFunctions::dft_filter(og_img, dft_filter_size, CVParams::LOW_PASS_FILTER);
      break;

    // OPTION 5
    // Apply a high pass filter to the Fourier spectrum and display the image
    // obtained after the inverse transform. The filter size will depend
    // of the value selected in the third slider
    case 5:
      processed_img = CVFunctions::dft_filter(og_img, dft_filter_size, CVParams::HIGH_PASS_FILTER);
      break;

    default:
      break;
  }

  cv::resize(processed_img, processed_img, cv::Size(), 0.5, 0.5);
  cv::imshow(CVParams::WINDOW_NAME, processed_img);
  cv::waitKey(1);

  return CVGroup(
    rgb,
    depth,
    disparity,
    left_rect,
    right_rect,
    left_raw,
    right_raw,
    in_pointcloud
  );
}

} // namespace computer_vision
