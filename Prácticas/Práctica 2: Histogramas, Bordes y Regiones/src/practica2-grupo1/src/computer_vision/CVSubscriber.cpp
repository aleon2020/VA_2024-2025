#include "computer_vision/CVSubscriber.hpp"

// GLOBAL PARAMETERS (NAMESPACE)

namespace CVParams
{

// Execution control
bool running = false;
bool callbacks_set = false;
bool histogram_window = false;

// Options for sliders
int option;
int hough_accumulator;
int area;

// Defining the name of the window and trackbars
inline std::string WINDOW_NAME = "Practice 2";
inline std::string WINDOW_HISTOGRAM_NAME = "Histograms";
inline std::string OPTION = "Option [0-5]";
inline std::string HOUGH_ACCUMULATOR = "Hough Accumulator [0-255]";
inline std::string AREA = "Area [0-20k]";
float img_scale = 1.0;

// Params for zoom_img() function
std::vector<cv::Point2f> points;

}

// COLOR SPACES (NAMESPACE)

namespace CVColors
{

// Pink color
inline std::vector<cv::Scalar> HSV_PINK = {cv::Scalar(140, 50, 150),
  cv::Scalar(170, 255, 255)};

}

// UTILITIES FUNCTIONS (NAMESPACE)

namespace CVUtils
{

cv::Mat draw_points(
  const cv::Mat img,
  const std::vector<cv::Point2f> points)
{
  cv::Mat dst = img.clone();
  int i = 1;

  if (points.empty()) {
    return dst;
  }

  for (const cv::Point2f & point : points) {
    cv::circle(dst, point, 5, cv::Scalar(0, 255, 255), -1);
    cv::putText(
          dst,
          std::to_string(i),
          cv::Point(point.x + 10, point.y - 10),
          cv::FONT_HERSHEY_SIMPLEX,
          1,
          cv::Scalar(0, 0, 255),
          2
    );
    i++;
  }

  return dst;
}

std::vector<cv::Vec2f> filter_similar_lines(
  const std::vector<cv::Vec2f> lines)
{
  std::vector<cv::Vec2f> filtered_lines;

  for (const auto & line : lines) {
    float rho = line[0], theta = line[1];
    bool is_similar = false;

    for (const auto & filtered_line : filtered_lines) {
      float filtered_rho = filtered_line[0], filtered_theta = filtered_line[1];
      if (std::abs(rho - filtered_rho) < 50 &&
        std::abs(theta - filtered_theta) < CV_PI / 90)
      {
        is_similar = true;
        break;
      }
    }

    if (!is_similar) {
      filtered_lines.push_back(line);
    }
  }

  return filtered_lines;
}

cv::Mat draw_lines(
  const cv::Mat img,
  const std::vector<cv::Vec2f> lines)
{
  cv::Mat dst = img.clone();
  int lines_detected = 0;
  float rho, theta;
  double a, b, x0, y0;

  for (const auto & line : lines) {
    rho = line[0];
    theta = line[1];
    a = cos(theta);
    b = sin(theta);
    x0 = a * rho;
    y0 = b * rho;

    cv::Point pt1(cvRound(x0 + 1000 * (-b)), cvRound(y0 + 1000 * (a)));
    cv::Point pt2(cvRound(x0 - 1000 * (-b)), cvRound(y0 - 1000 * (a)));
    cv::line(dst, pt1, pt2, cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

    lines_detected++;
  }

  cv::putText(
    dst,
    "Lines detected: " + std::to_string(lines_detected),
    cv::Point(10, 30),
    cv::FONT_HERSHEY_SIMPLEX,
    1,
    cv::Scalar(0, 0, 255),
    2
  );

  return dst;
}

cv::Mat draw_contours(
  const cv::Mat img,
  const std::vector<std::vector<cv::Point>> contours,
  const int area)
{
  cv::Mat dst = img.clone();
  double contour_area;
  int cx, cy;

  if (contours.empty()) {
    return dst;
  }

  for (const auto & contour : contours) {
    contour_area = cv::contourArea(contour);
    if (contour_area > area) {
      cv::Scalar random_color(
        cv::theRNG().uniform(0, 256),
        cv::theRNG().uniform(0, 256),
        cv::theRNG().uniform(0, 256)
      );

      cv::drawContours(dst, std::vector<std::vector<cv::Point>>{contour}, -1,
          random_color, 2, cv::LINE_AA);

      cv::Moments moments = cv::moments(contour);
      if (moments.m00 != 0) {
        cx = static_cast<int>(moments.m10 / moments.m00);
        cy = static_cast<int>(moments.m01 / moments.m00);

        cv::circle(dst, cv::Point(cx, cy), 5, cv::Scalar(0, 0, 255), -1);

        cv::putText(
          dst,
          std::to_string(static_cast<int>(contour_area)),
          cv::Point(cx + 10, cy - 10),
          cv::FONT_HERSHEY_SIMPLEX,
          0.5,
          cv::Scalar(255, 255, 255),
          1
        );
      }
    }
  }

  return dst;
}

std::vector<std::vector<cv::Point>> filter_contours(
  const std::vector<std::vector<cv::Point>> contours)
{
  std::vector<std::vector<cv::Point>> filtered_contours;

  for (const auto & contour : contours) {
    cv::RotatedRect bounding_box = cv::minAreaRect(contour);
    float aspect_ratio = std::max(bounding_box.size.width,
        bounding_box.size.height) /
      std::min(bounding_box.size.width, bounding_box.size.height);

    // Check if the contour has 4 vertices when approximated
    std::vector<cv::Point> approx;
    cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true),
        true);

    // Only pass contours that are rectangles (4 vertices) and have a significant aspect ratio
    if (approx.size() == 4 && aspect_ratio > 1.25) {
      filtered_contours.push_back(contour);
    }
  }

  return filtered_contours;
}

}

// IMAGE PROCESSING FUNCTIONS (NAMESPACE)

namespace CVFunctions
{

cv::Mat zoom_img(
  const cv::Mat img,
  const std::vector<cv::Point2f> points)
{
  cv::Mat dst;
  cv::Mat src = img.clone();
  cv::Point2f src_vertices[4];
  cv::Point2f dst_vertices[4];

  if (points.size() < 4) {
    return CVUtils::draw_points(src, points);
  }

  src_vertices[0] = points[0];
  src_vertices[1] = points[1];
  src_vertices[2] = points[2];
  src_vertices[3] = points[3];

  dst_vertices[0] = cv::Point2f(0, 0);
  dst_vertices[1] = cv::Point2f(src.cols, 0);
  dst_vertices[2] = cv::Point2f(0, src.rows);
  dst_vertices[3] = cv::Point2f(src.cols, src.rows);

  cv::Mat M = cv::getPerspectiveTransform(src_vertices, dst_vertices);
  cv::warpPerspective(src, dst, M, cv::Size(src.cols, src.rows));

  return dst;
}

cv::Mat find_pink_balls(
  const cv::Mat img)
{
  cv::Mat hsv;
  cv::Mat mask;
  cv::Mat gray;
  cv::Mat processed_img = img.clone();
  std::vector<cv::Vec3f> circles;

  cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);
  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
  cv::GaussianBlur(hsv, hsv, cv::Size(9, 9), 2, 2);
  cv::medianBlur(hsv, hsv, 5);

  cv::inRange(hsv, CVColors::HSV_PINK[0], CVColors::HSV_PINK[1], mask);

  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
  cv::dilate(mask, mask, kernel, cv::Point(-1, -1), 2);
  cv::erode(mask, mask, kernel, cv::Point(-1, -1), 2);

  cv::bitwise_and(gray, mask, gray);
  cv::GaussianBlur(gray, gray, cv::Size(9, 9), 2, 2);

  cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1, mask.rows / 32, 50, 15,
      5, 0);

  for (const auto & circle : circles) {
    cv::Point center(cvRound(circle[0]), cvRound(circle[1]));
    int radius = cvRound(circle[2]);
    cv::circle(processed_img, center, 3, cv::Scalar(0, 255, 0), -1);
    cv::circle(processed_img, center, radius, cv::Scalar(255, 0, 0), 2);
  }

  return processed_img;
}

cv::Mat find_lines(
  const cv::Mat img,
  const int hough_accumulator)
{
  cv::Mat gray;
  cv::Mat edges;
  cv::Mat mask;
  cv::Mat processed_img = img.clone();
  std::vector<cv::Vec2f> lines;
  std::vector<cv::Vec2f> filtered_lines;

  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

  cv::threshold(gray, mask, 100, 255, cv::THRESH_BINARY_INV);

  cv::Canny(mask, edges, 50, 150, 3);

  cv::HoughLines(edges, lines, 1, CV_PI / 180, hough_accumulator);

  filtered_lines = CVUtils::filter_similar_lines(lines);

  processed_img = CVUtils::draw_lines(processed_img, filtered_lines);

  return processed_img;
}

cv::Mat find_rectangles(
  const cv::Mat img,
  const int area)
{
  cv::Mat gray;
  cv::Mat edges;
  cv::Mat mask;
  cv::Mat processed_img = img.clone();
  std::vector<std::vector<cv::Point>> contours;
  std::vector<std::vector<cv::Point>> filtered_contours;
  std::vector<cv::Vec4i> hierarchy;

  cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

  cv::threshold(gray, mask, 100, 255, cv::THRESH_BINARY_INV);

  cv::Canny(mask, edges, 50, 150, 3);

  cv::findContours(edges, contours, hierarchy, cv::RETR_TREE,
      cv::CHAIN_APPROX_SIMPLE);

  filtered_contours = CVUtils::filter_contours(contours);

  processed_img = CVUtils::draw_contours(processed_img, filtered_contours,
      area);

  return processed_img;
}

cv::Mat calculateHistograms(
  const cv::Mat & img, cv::Mat hist[3],
  cv::Mat equalized[3], cv::Mat equalized_hist[3])
{
  cv::Mat bgr[3];
  cv::split(img, bgr);

  int histSize = 256;
  float range[] = {0, 256};
  const float * histRange = {range};

  for (int i = 0; i < 3; i++) {
    cv::calcHist(&bgr[i], 1, 0, cv::Mat(), hist[i], 1, &histSize, &histRange,
        true, false);
    cv::equalizeHist(bgr[i], equalized[i]);
    cv::calcHist(&equalized[i], 1, 0, cv::Mat(), equalized_hist[i], 1,
        &histSize, &histRange, true, false);
  }

  cv::Mat combined;
  cv::merge(equalized, 3, combined);
  return combined;
}

cv::Mat drawHistograms(const cv::Mat hist[3], const cv::Mat equalized_hist[3])
{
  int hist_w = 512, hist_h = 400;
  cv::Mat hist_image(hist_h, hist_w, CV_8UC3, cv::Scalar(0, 0, 0));

  int histSize = 256;
  cv::Mat hist_norm[3], equalized_hist_norm[3];
  for (int i = 0; i < 3; i++) {
    cv::normalize(hist[i], hist_norm[i], 0, hist_image.rows, cv::NORM_MINMAX,
        -1, cv::Mat());
    cv::normalize(equalized_hist[i], equalized_hist_norm[i], 0, hist_image.rows,
        cv::NORM_MINMAX, -1, cv::Mat());
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 1; j < histSize; j++) {
      cv::line(hist_image,
              cv::Point(hist_w / histSize * (j - 1),
          hist_h - cvRound(hist_norm[i].at<float>(j - 1))),
              cv::Point(hist_w / histSize * (j),
          hist_h - cvRound(hist_norm[i].at<float>(j))),
              cv::Scalar((i == 0) ? 255 : 0, (i == 1) ? 255 : 0,
        (i == 2) ? 255 : 0), 2, cv::LINE_AA);

      cv::line(hist_image,
              cv::Point(hist_w / histSize * (j - 1),
          hist_h - cvRound(equalized_hist_norm[i].at<float>(j - 1))),
              cv::Point(hist_w / histSize * (j),
          hist_h - cvRound(equalized_hist_norm[i].at<float>(j))),
              cv::Scalar((i == 0) ? 128 : 0, (i == 1) ? 128 : 0,
        (i == 2) ? 128 : 0), 1, cv::LINE_AA);
    }
  }

  return hist_image;
}

void
annotateHistogram(
  cv::Mat & hist_image, const cv::Mat hist[3],
  const cv::Mat equalized_hist[3], const cv::Mat & img)
{
  double correlation[3];
  int average_brightness = 0;
  int histSize = 256;

  for (int i = 0; i < 3; i++) {
    correlation[i] = cv::compareHist(hist[i], equalized_hist[i],
        cv::HISTCMP_CORREL);
    cv::putText(hist_image,
          "Correlation: " + std::to_string(correlation[i]),
          cv::Point(10, 30 + i * 25),
          cv::FONT_HERSHEY_SIMPLEX,
          0.7,
          cv::Scalar((i == 0) ? 255 : 0, (i == 1) ? 255 : 0,
      (i == 2) ? 255 : 0),
          2);
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < histSize; j++) {
      average_brightness += j * hist[i].at<float>(j);
    }
  }
  average_brightness /= (3 * img.total());

  std::string exposure_text;
  if (average_brightness < 65) {
    exposure_text = "Subexpuesta";
  } else if (average_brightness > 190) {
    exposure_text = "Sobreexpuesta";
  } else {
    exposure_text = "Bien expuesta";
  }

  cv::putText(hist_image, exposure_text, cv::Point(10, 105),
      cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
}

cv::Mat show_histograms(const cv::Mat img)
{
  cv::Mat hist[3], equalized[3], equalized_hist[3];
  cv::Mat combined = calculateHistograms(img, hist, equalized, equalized_hist);

  cv::Mat hist_image = drawHistograms(hist, equalized_hist);

  annotateHistogram(hist_image, hist, equalized_hist, img);

  cv::imshow(CVParams::WINDOW_HISTOGRAM_NAME, hist_image);

  return combined;
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
  cv::createTrackbar(CVParams::HOUGH_ACCUMULATOR, CVParams::WINDOW_NAME,
    nullptr, 255, 0);
  cv::createTrackbar(CVParams::AREA, CVParams::WINDOW_NAME, nullptr, 20000, 0);
  cv::setTrackbarPos(CVParams::OPTION, CVParams::WINDOW_NAME, 0);
  cv::setTrackbarPos(CVParams::HOUGH_ACCUMULATOR, CVParams::WINDOW_NAME, 123);
  cv::setTrackbarPos(CVParams::AREA, CVParams::WINDOW_NAME, 250);
}

// on_mouse() function
// Create mouse callback
void
on_mouse(int event, int x, int y, int, void * userdata)
{
  std::vector<cv::Point2f> * points = nullptr;
  float scale = 1 / CVParams::img_scale;

  if (event != cv::EVENT_LBUTTONDOWN) {
    return;
  }

  points = static_cast<std::vector<cv::Point2f> *>(userdata);

  if (points == nullptr) {
    return;
  }

  points->push_back(cv::Point2f(x * scale, y * scale));

  if (points->size() > 4) {
    points->clear();
  }
}

void
handleHistogramWindow()
{
  if (CVParams::option == 2) {
    if (!CVParams::histogram_window) {
      cv::namedWindow(CVParams::WINDOW_HISTOGRAM_NAME);
      CVParams::histogram_window = true;
    }
  } else if (CVParams::histogram_window) {
    cv::destroyWindow(CVParams::WINDOW_HISTOGRAM_NAME);
    CVParams::histogram_window = false;
  }
}

void
handleMouseCallback()
{
  if (CVParams::option == 1) {
    if (!CVParams::callbacks_set) {
      CVParams::points.clear();
      cv::setMouseCallback(CVParams::WINDOW_NAME, on_mouse, &CVParams::points);
      CVParams::callbacks_set = true;
    }
  } else if (CVParams::callbacks_set) {
    cv::setMouseCallback(CVParams::WINDOW_NAME, nullptr, nullptr);
    CVParams::callbacks_set = false;
  }
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

  CVParams::option = cv::getTrackbarPos(CVParams::OPTION,
      CVParams::WINDOW_NAME);
  CVParams::hough_accumulator = cv::getTrackbarPos(CVParams::HOUGH_ACCUMULATOR,
      CVParams::WINDOW_NAME);
  CVParams::area = cv::getTrackbarPos(CVParams::AREA, CVParams::WINDOW_NAME);

  handleHistogramWindow();
  handleMouseCallback();

  switch(CVParams::option) {
    case 0:
      processed_img = og_img;
      break;
    case 1:
      processed_img = CVFunctions::zoom_img(og_img, CVParams::points);
      break;
    case 2:
      processed_img = CVFunctions::show_histograms(og_img);
      break;
    case 3:
      processed_img = CVFunctions::find_lines(og_img,
        CVParams::hough_accumulator);
      break;
    case 4:
      processed_img = CVFunctions::find_pink_balls(og_img);
      break;
    case 5:
      processed_img = CVFunctions::find_rectangles(og_img, CVParams::area);
      break;
    default:
      break;
  }

  cv::resize(processed_img, processed_img, cv::Size(), CVParams::img_scale,
      CVParams::img_scale);
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
