#include "computer_vision/CVSubscriber.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// GLOBAL PARAMETERS (NAMESPACE)

namespace CVParams
{

// Execution control
bool running = false;

// Defining the name of the window and trackbars
inline std::string WINDOW_NAME = "Final Practice";
inline std::string OPTION = "Option [0-1]";
inline std::string SEGMENTATION_MODE_TXT = "Segmentation Mode [0-1]";
inline std::string FRAMES_AHEAD = "Frames ahead [0-30]";
inline std::string CLUSTERS = "Number of Clusters [1-10]";
inline std::string OPTICAL_FLOW = "Optical Flow Mode [0-1]";
float img_scale = 0.5;

typedef enum _segmentationMode
{
  K_MEANS = 0,
  IN_RANGE
} SEGMENTATION_MODE;

typedef enum _opticalFlowMode
{
  LK = 0,
  CENTER_OF_MASS
} OPTICAL_FLOW_MODE;

// Optical flow parameters
cv::Mat previous_gray_frame;
std::vector<cv::Point2f> previous_points;
bool tracking_initialized = false;
std::deque<cv::Point2f> displacement_history;
const int MAX_HISTORY = 10;
// K-Means parameters
cv::Scalar lower_hsv_bound = cv::Scalar(90, 60, 0);
cv::Scalar upper_hsv_bound = cv::Scalar(140, 255, 255);

// Morphological kernel size
int morph_kernel_size = 5;

cv::Mat prev_rgb;

}

// COLOR SPACES (NAMESPACE)

namespace CVColors
{

}

// UTILITIES FUNCTIONS (NAMESPACE)

namespace CVUtils
{

cv::Mat inRangeSegmentation(const cv::Mat & frame)
{
  if (frame.empty()) {
    std::cerr << "[ERROR] Empty input frame in inRangeSegmentation." <<
      std::endl;
    return cv::Mat::zeros(frame.size(), CV_8UC1);
  }

  cv::Mat hsv_frame;
  cv::cvtColor(frame, hsv_frame, cv::COLOR_BGR2HSV);

  cv::Mat mask;
  cv::inRange(hsv_frame, CVParams::lower_hsv_bound, CVParams::upper_hsv_bound,
      mask);

  int morph_size = std::max(1, CVParams::morph_kernel_size);
  cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_RECT,
      cv::Size(morph_size, morph_size));

  if (morph_size > 1) {
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel);
  }

  return mask;
}


cv::Mat kmeansSegmentation(const cv::Mat & frame)
{
  int cluster_count = cv::getTrackbarPos(CVParams::CLUSTERS,
      CVParams::WINDOW_NAME) + 1;

  cv::Mat hsv;
  cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

  cv::Mat samples = hsv.reshape(1, hsv.rows * hsv.cols);
  samples.convertTo(samples, CV_32F);

  cv::Mat labels, centers;
  cv::kmeans(samples, cluster_count, labels,
              cv::TermCriteria(cv::TermCriteria::EPS +
      cv::TermCriteria::MAX_ITER, 10, 1.0),
              3, cv::KMEANS_PP_CENTERS, centers);

  const float hue_center = (CVParams::lower_hsv_bound[0] +
    CVParams::upper_hsv_bound[0]) / 2.0f;
  const float hue_range = (CVParams::upper_hsv_bound[0] -
    CVParams::lower_hsv_bound[0]) / 2.0f;
  const float score_threshold = 0.6f;

  int target_cluster_idx = -1;
  float best_score = -1.0f;

  for (int i = 0; i < centers.rows; ++i) {
    float hue = centers.at<float>(i, 0);
    float sat = centers.at<float>(i, 1);
      // float val = centers.at<float>(i, 2); // Not used in scoring

    float hue_diff = std::abs(hue - hue_center);
    float hue_score = 1.0f - (hue_diff / hue_range);
    hue_score = std::max(0.0f, std::min(hue_score, 1.0f));

    float sat_score = sat / 255.0f;
    float final_score = (hue_score * 0.8f) + (sat_score * 0.2f);

    if (final_score > best_score) {
      best_score = final_score;
      target_cluster_idx = i;
    }
  }

  if (best_score < score_threshold) {
    std::cerr << "[INFO] No confident cluster found (best_score = " <<
      best_score << "). No ball detected." << std::endl;
    return cv::Mat::zeros(frame.size(), CV_8UC1);
  }

  std::cout << "[INFO] Ball detected in cluster " << target_cluster_idx <<
    " (score = " << best_score << ")" << std::endl;

  cv::Mat mask(hsv.rows, hsv.cols, CV_8UC1, cv::Scalar(0));

  for (int i = 0; i < labels.rows; ++i) {
    int label = labels.at<int>(i);
    if (label == target_cluster_idx) {
      mask.at<uchar>(i / hsv.cols, i % hsv.cols) = 255;
    }
  }

  return mask;
}

std::vector<cv::Vec3f> detectCircles(const cv::Mat & binary_mask)
{
  std::vector<cv::Vec3f> detected_circles;

  if (binary_mask.empty() || binary_mask.type() != CV_8UC1) {
    std::cerr << "[ERROR] Invalid binary mask input to detectCircles." <<
      std::endl;
    return detected_circles;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_mask.clone(), contours, cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    if (contour.size() < 5) {continue;}

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);

    if (radius > 5.0f) {
      detected_circles.emplace_back(center.x, center.y, radius);
    }
  }

  return detected_circles;
}

std::vector<cv::Point2f> generateCircleEdgePoints(
  const cv::Point2f & center,
  float radius, int num_points = 8)
{
  std::vector<cv::Point2f> points;
  const float angle_step = 2.0f * CV_PI / static_cast<float>(num_points);

  for (int i = 0; i < num_points; ++i) {
    float angle = i * angle_step;
    float x = center.x + radius * std::cos(angle);
    float y = center.y + radius * std::sin(angle);
    points.emplace_back(x, y);
  }

  points.emplace_back(center);

  return points;
}

// preprocessFrame() function
// Preprocess the frame for feature detections
cv::Mat
preprocessFrame(const cv::Mat & input)
{
  cv::Mat processed;

  // Convert to grayscale
  if (input.channels() > 1) {
    cv::cvtColor(input, processed, cv::COLOR_BGR2GRAY);
  } else {
    processed = input.clone();
  }

  // Apply histogram equalization
  cv::equalizeHist(processed, processed);

  return processed;
}

std::vector<cv::Point2f> getBallPoints(const cv::Mat & frame)
{
  std::vector<cv::Point2f> points;
  cv::Mat mask = CVUtils::inRangeSegmentation(frame);
  std::vector<cv::Vec3f> circles = CVUtils::detectCircles(mask);

  if (!circles.empty()) {
    const auto & circle = circles[0];
    cv::Point2f center(circle[0], circle[1]);
    int radius = static_cast<int>(circle[2]);
    points = CVUtils::generateCircleEdgePoints(center, radius / 2, 8);
  } else {
    std::cerr << "[INFO] No ball detected." << std::endl;
  }

  if (points.empty()) {
    std::cerr << "[INFO] No points generated." << std::endl;
  }

  return points;
}

}

// IMAGE PROCESSING FUNCTIONS (NAMESPACE)

namespace CVFunctions
{

// processOpticalFlow() function
// Process the optical flow
cv::Mat processOpticalFlow(
  const cv::Mat & frame,
  const CVParams::OPTICAL_FLOW_MODE mode)
{
  std::vector<cv::Point2f> next_points, good_new_points;
  cv::Mat processed_frame = frame.clone();
  cv::Mat mask = CVUtils::inRangeSegmentation(frame);
  std::vector<cv::Vec3f> circles = CVUtils::detectCircles(mask);
  int radius = 0;
  cv::Point2f center(0.f, 0.f);
  if (!circles.empty()) {
    center = cv::Point2f(circles[0][0], circles[0][1]);
    radius = static_cast<int>(circles[0][2]);
  }
  cv::Mat masked_frame;
  cv::bitwise_and(frame, frame, masked_frame, mask);
  cv::Mat current_gray = CVUtils::preprocessFrame(masked_frame);

  if (!CVParams::tracking_initialized) {
    CVParams::previous_gray_frame = current_gray.clone();
    CVParams::previous_points = CVUtils::getBallPoints(frame);
    CVParams::tracking_initialized = true;
    return processed_frame;
  }

  if (CVParams::previous_points.empty()) {
    std::cerr <<
      "[ERROR] No points detected for optical flow initialization." <<
      std::endl;
    CVParams::tracking_initialized = false;
    return processed_frame;
  }

  // Optical Flow
  std::vector<uchar> status;
  std::vector<float> err;
  cv::Size win_size(41, 41);
  int max_level = 5;
  cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50,
    0.03);

  cv::calcOpticalFlowPyrLK(
      CVParams::previous_gray_frame, current_gray,
      CVParams::previous_points, next_points,
      status, err,
      win_size, max_level, criteria
  );

  cv::Point2f total_displacement(0.f, 0.f);
  cv::Point2f center_current(0.f, 0.f);
  int valid_points = 0;

  for (size_t i = 0; i < CVParams::previous_points.size(); ++i) {
    if (status[i]) {
      good_new_points.push_back(next_points[i]);
      cv::circle(processed_frame, next_points[i], 5, cv::Scalar(0, 255, 0), -1);
      cv::line(processed_frame, CVParams::previous_points[i], next_points[i],
          cv::Scalar(0, 255, 0), 2);

      total_displacement += (next_points[i] - CVParams::previous_points[i]);
      center_current += next_points[i];
      valid_points++;
    }
  }

  if (valid_points > 0) {
    total_displacement *= (1.0f / valid_points);
    center_current *= (1.0f / valid_points);

    CVParams::displacement_history.push_back(total_displacement);
    if (CVParams::displacement_history.size() > CVParams::MAX_HISTORY) {
      CVParams::displacement_history.pop_front();
    }

    cv::Point2f mean_displacement(0.f, 0.f);
    for (const auto & d : CVParams::displacement_history) {
      mean_displacement += d;
    }
    mean_displacement *= (1.0f / CVParams::displacement_history.size());

    cv::arrowedLine(
        processed_frame,
        center_current,
        center_current + mean_displacement * 5.0f,
        cv::Scalar(0, 0, 255), 3, cv::LINE_AA
    );

    int future_frames = cv::getTrackbarPos(CVParams::FRAMES_AHEAD,
        CVParams::WINDOW_NAME);
    cv::Point2f predicted_position = center_current + mean_displacement *
      future_frames;

    cv::circle(processed_frame, predicted_position, radius,
        cv::Scalar(255, 0, 0), 3);
  } else {
    std::cerr << "[WARNING] No valid points tracked for movement vector." <<
      std::endl;
  }

  CVParams::previous_points = good_new_points;
  CVParams::previous_gray_frame = current_gray.clone();

  return processed_frame;
}


std::vector<cv::Vec3f> detectCircles(const cv::Mat & binary_mask)
{
  std::vector<cv::Vec3f> detected_circles;

  if (binary_mask.empty() || binary_mask.type() != CV_8UC1) {
    std::cerr << "[ERROR] Invalid binary mask input to detectCircles." <<
      std::endl;
    return detected_circles;
  }

  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_mask.clone(), contours, cv::RETR_EXTERNAL,
      cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    if (contour.size() < 5) {continue;}

    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contour, center, radius);

    if (radius > 5.0f) {
      detected_circles.emplace_back(center.x, center.y, radius);
    }
  }

  return detected_circles;
}


// processColorSegmentation() function
// Process the color segmentation
cv::Mat processColorSegmentation(
  const cv::Mat & frame,
  const CVParams::SEGMENTATION_MODE mode)
{
  if (frame.empty()) {
    std::cerr << "No frame available for color segmentation!" << std::endl;
    return frame;
  }

  cv::Mat mask, processed_frame = frame.clone();
  std::vector<cv::Vec3f> circles;

  switch (mode) {
    case CVParams::SEGMENTATION_MODE::K_MEANS:
      mask = CVUtils::kmeansSegmentation(frame);
      break;
    case CVParams::SEGMENTATION_MODE::IN_RANGE:
      mask = CVUtils::inRangeSegmentation(frame);
      break;
    default:
      std::cerr << "Invalid segmentation mode!" << std::endl;
      return frame;
  }

  circles = detectCircles(mask);

  if (!circles.empty()) {
    for (const auto & circle : circles) {
      cv::Point center(static_cast<int>(circle[0]),
        static_cast<int>(circle[1]));
      int radius = static_cast<int>(circle[2]);

      cv::Rect bounding_box(center.x - radius, center.y - radius, radius * 2,
        radius * 2);
      cv::rectangle(processed_frame, bounding_box, cv::Scalar(255, 0, 0), 2);

      std::string label = "Ball";
      cv::Point text_origin(bounding_box.x, bounding_box.y - 5);

      double font_scale = 2.0;
      int thickness = 1;
      int baseline = 0;
      cv::Size text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX,
          font_scale, thickness, &baseline);
      if (text_origin.y - text_size.height < 0) {
        text_origin.y = bounding_box.y + text_size.height + 5;
      }

      cv::putText(processed_frame, label, text_origin, cv::FONT_HERSHEY_SIMPLEX,
          font_scale, cv::Scalar(255, 0, 0), thickness);
    }
  }

  return processed_frame;
}

}

// init_window() function
// Initialize the window and trackbars
void
initWindow()
{
  if (CVParams::running) {
    return;
  }

  CVParams::running = true;
  cv::namedWindow(CVParams::WINDOW_NAME);

  // Create trackbars with callback functions

  cv::createTrackbar(CVParams::OPTION,
                     CVParams::WINDOW_NAME,
                     nullptr, 1, nullptr);

  cv::createTrackbar(CVParams::SEGMENTATION_MODE_TXT,
                     CVParams::WINDOW_NAME,
                     nullptr, 1, nullptr);

  cv::createTrackbar(CVParams::CLUSTERS,
                     CVParams::WINDOW_NAME,
                     nullptr, 10, nullptr);

  cv::createTrackbar(CVParams::FRAMES_AHEAD,
                     CVParams::WINDOW_NAME,
                     nullptr, 30, nullptr);

  // cv::createTrackbar(CVParams::OPTICAL_FLOW,
  //                    CVParams::WINDOW_NAME,
  //                    nullptr, 1, nullptr);

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
  cv::Mat processed_frame;

  if (rgb.empty()) {
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

  if (CVParams::prev_rgb.empty()) {
    // Primera vez, no podemos comparar
    CVParams::prev_rgb = rgb.clone();
  } else {
    double error = cv::norm(rgb, CVParams::prev_rgb, cv::NORM_L1);

    if (error == 0.0) {
      std::cout << "[INFO] Frame has not changed." << std::endl;
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
    } else {
      std::cout << "[INFO] New frame detected!" << std::endl;
        // Actualizar prev_rgb para la siguiente comparación
      CVParams::prev_rgb = rgb.clone();
    }
  }


  initWindow();

  processed_frame = rgb;

  // Get current trackbar values
  int option = cv::getTrackbarPos(CVParams::OPTION, CVParams::WINDOW_NAME);
  int segmentation_mode = cv::getTrackbarPos(CVParams::SEGMENTATION_MODE_TXT,
      CVParams::WINDOW_NAME);
  int optical_flow_mode = cv::getTrackbarPos(CVParams::OPTICAL_FLOW,
      CVParams::WINDOW_NAME);

  switch (option) {
    case 0:
      processed_frame = CVFunctions::processColorSegmentation(rgb,
        static_cast<CVParams::SEGMENTATION_MODE>(segmentation_mode));
      break;
    case 1:
      processed_frame = CVFunctions::processOpticalFlow(rgb,
        static_cast<CVParams::OPTICAL_FLOW_MODE>(optical_flow_mode));
      break;
    default:
      std::cerr << "[ERROR] Invalid option selected!" << std::endl;
      break;
  }

  cv::resize(processed_frame, processed_frame, cv::Size(), CVParams::img_scale,
      CVParams::img_scale);
  cv::imshow(CVParams::WINDOW_NAME, processed_frame);
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
