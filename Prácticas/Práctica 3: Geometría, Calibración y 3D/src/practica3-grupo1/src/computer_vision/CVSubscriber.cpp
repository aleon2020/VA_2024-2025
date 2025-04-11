#include "computer_vision/CVSubscriber.hpp"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

// GLOBAL PARAMETERS (NAMESPACE)

namespace CVParams
{

// Execution control
bool running = false;

// Options for sliders
int option;

// Defining the name of the window and trackbars
inline std::string WINDOW_NAME = "Practice 3";
inline std::string OPTION = "Option [0-6]";
float img_scale = 0.5;

const cv::Size CHESSBOARD_SIZE = cv::Size(9, 6);
const float SQUARE_SIZE = 24.0f; // milimeters

cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1502.457763671875, 0,
  949.13916015625, 0,
  1501.9019775390625, 546.2084350585938, 0, 0, 1);
cv::Mat dist_coeffs = cv::Mat::zeros(5, 1, CV_64F);
std::vector<std::vector<cv::Point2f>> image_points;
std::vector<std::vector<cv::Point3f>> object_points;

bool show_rectified = false;

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

std::vector<cv::Point3f>
generate_3D_chessboard_corners()
{
  std::vector<cv::Point3f> corners;

  for (int i = 0; i < CVParams::CHESSBOARD_SIZE.height; i++) {
    for (int j = 0; j < CVParams::CHESSBOARD_SIZE.width; j++) {
      corners.push_back(cv::Point3f(j * CVParams::SQUARE_SIZE,
          i * CVParams::SQUARE_SIZE, 0));
    }
  }

  return corners;
}

void
save_calibration(const std::string & filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  if (fs.isOpened()) {
    fs << "camera_matrix" << CVParams::camera_matrix;
    fs << "distortion_coefficients" << CVParams::dist_coeffs;
    fs.release();
    std::cout << "Calibration saved to " << filename << std::endl;
  } else {
    std::cerr << "Error: Could not open file to save calibration" << std::endl;
  }
}

bool
load_calibration(const std::string & filename)
{
  cv::FileStorage fs(filename, cv::FileStorage::READ);

  if (fs.isOpened()) {
    fs["camera_matrix"] >> CVParams::camera_matrix;
    fs["distortion_coefficients"] >> CVParams::dist_coeffs;
    fs.release();
    std::cout << "Calibration loaded from " << filename << std::endl;
    return true;
  } else {
    std::cerr << "Error: Could not open file to load calibration." << std::endl;
    return false;
  }

  return false;
}

void
perform_calibration(cv::Size image_size)
{
  std::vector<cv::Mat> rvecs, tvecs;
  int flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_FIX_K3 |
    cv::CALIB_ZERO_TANGENT_DIST |
    cv::CALIB_FIX_PRINCIPAL_POINT;
  cv::calibrateCamera(CVParams::object_points, CVParams::image_points,
      image_size,
      CVParams::camera_matrix, CVParams::dist_coeffs, rvecs, tvecs, flags);
}

std::vector<cv::Point3f>
create3DCube(float size = 96.0f, float height = -96.0f)
{
  std::vector<cv::Point3f> cube;
  float halfSize = size / 2.0f;
  float centerX = (CVParams::CHESSBOARD_SIZE.width - 1) *
    CVParams::SQUARE_SIZE / 2.0f;
  float centerY = (CVParams::CHESSBOARD_SIZE.height - 1) *
    CVParams::SQUARE_SIZE / 2.0f;
  cube.push_back(cv::Point3f(centerX - halfSize, centerY - halfSize, 0));
  cube.push_back(cv::Point3f(centerX + halfSize, centerY - halfSize, 0));
  cube.push_back(cv::Point3f(centerX + halfSize, centerY + halfSize, 0));
  cube.push_back(cv::Point3f(centerX - halfSize, centerY + halfSize, 0));
  cube.push_back(cv::Point3f(centerX - halfSize, centerY - halfSize, height));
  cube.push_back(cv::Point3f(centerX + halfSize, centerY - halfSize, height));
  cube.push_back(cv::Point3f(centerX + halfSize, centerY + halfSize, height));
  cube.push_back(cv::Point3f(centerX - halfSize, centerY + halfSize, height));
  return cube;
}

}

// IMAGE PROCESSING FUNCTIONS (NAMESPACE)

namespace CVFunctions
{

cv::Mat get_chessboard_img(const cv::Mat rgb)
{
  cv::Mat chessboard_img = rgb.clone();
  cv::Mat chessboard_rectified;
  cv::Mat gray_img;
  cv::cvtColor(chessboard_img, gray_img, cv::COLOR_BGR2GRAY);
  std::vector<cv::Point2f> corners;
  bool found = false;
  int key = 0;


  found = cv::findChessboardCorners(chessboard_img, CVParams::CHESSBOARD_SIZE,
    corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
    cv::CALIB_CB_NORMALIZE_IMAGE);

  if (found) {
    cv::cornerSubPix(gray_img, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
        0.001));
    cv::drawChessboardCorners(chessboard_img, CVParams::CHESSBOARD_SIZE,
        corners,
        found);
  } else {
    return chessboard_img;
  }

  key = cv::waitKey(1);

  if (key == 's') {
    CVUtils::save_calibration("calibration.yml");
  } else if (key == 'c') {
    CVParams::object_points.push_back(
        CVUtils::generate_3D_chessboard_corners());
    CVParams::image_points.push_back(corners);
    CVUtils::perform_calibration(chessboard_img.size());
    std::cout << "Calibration performed" << std::endl;
    std::cout << "Camera matrix: " << CVParams::camera_matrix << std::endl;
    std::cout << "Distortion coefficients: " << CVParams::dist_coeffs <<
      std::endl;
    std::cout << "Press 's' to save calibration" << std::endl;
  } else if (key == 'a') {
    if (CVParams::show_rectified) {
      CVParams::show_rectified = false;
      std::cout << "Undistortion disabled" << std::endl;
    } else {
      if (CVUtils::load_calibration("calibration.yml")) {
        CVParams::show_rectified = true;
        std::cout << "Undistortion enabled" << std::endl;
      }
    }
  }

  if (!CVParams::show_rectified) {
    return chessboard_img;
  }

  cv::undistort(chessboard_img, chessboard_rectified,
      CVParams::camera_matrix, CVParams::dist_coeffs);

  return chessboard_rectified;
}

cv::Mat draw_cube(
  const cv::Mat & image)
{
  cv::Mat cube_img = image.clone();
  std::vector<cv::Point3f> cube_points = CVUtils::create3DCube();
  std::vector<cv::Point3f> chessboard_points =
    CVUtils::generate_3D_chessboard_corners();
  std::vector<cv::Point2f> corners;
  bool found = cv::findChessboardCorners(cube_img, CVParams::CHESSBOARD_SIZE,
      corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
      cv::CALIB_CB_NORMALIZE_IMAGE);

  if (found) {
    cv::Mat gray_img;
    cv::cvtColor(cube_img, gray_img, cv::COLOR_BGR2GRAY);
    cv::cornerSubPix(gray_img, corners, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30,
        0.001));
    cv::Mat rvec, tvec;
    cv::solvePnP(chessboard_points, corners, CVParams::camera_matrix,
        CVParams::dist_coeffs, rvec, tvec);
    std::vector<cv::Point2f> cube2D;
    cv::projectPoints(cube_points, rvec, tvec, CVParams::camera_matrix,
        CVParams::dist_coeffs, cube2D);
    for (int i = 0; i < 4; i++) {
      cv::line(cube_img, cube2D[i], cube2D[(i + 1) % 4], cv::Scalar(0, 0, 255),
          2);
    }
    for (int i = 4; i < 8; i++) {
      cv::line(cube_img, cube2D[i], cube2D[i - 4], cv::Scalar(0, 0, 255), 2);
    }
    for (int i = 4; i < 8; i++) {
      cv::line(cube_img, cube2D[i], cube2D[(i + 1) % 4 + 4],
          cv::Scalar(0, 0, 255),
               2);
    }
  } else {
    std::cout << "Chessboard not found" << std::endl;
    return cube_img;
  }

  return cube_img;
}

cv::Mat get_disparity_map(const cv::Mat left_rect, const cv::Mat right_rect)
{
  cv::Mat disparity, disparity_norm;
  int num_disparities = 64;
  int block_size = 15;

  cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(num_disparities,
      block_size);

  stereo->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
  stereo->setPreFilterSize(9);
  stereo->setPreFilterCap(31);
  stereo->setTextureThreshold(10);
  stereo->setUniquenessRatio(15);
  stereo->setSpeckleWindowSize(100);
  stereo->setSpeckleRange(32);
  stereo->setDisp12MaxDiff(1);

  stereo->compute(left_rect, right_rect, disparity);

  cv::normalize(disparity, disparity_norm, 0, 255, cv::NORM_MINMAX, CV_8U);

  return disparity_norm;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate_point_cloud(
  const cv::Mat & rgb,
  const cv::Mat & depth,
  const cv::Mat & K // Matriz intrínseca de 3x3
)
{
  float fx = K.at<double>(0, 0);
  float fy = K.at<double>(1, 1);
  float cx = K.at<double>(0, 2);
  float cy = K.at<double>(1, 2);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  cloud->width = rgb.cols;
  cloud->height = rgb.rows;
  cloud->is_dense = false;
  cloud->points.reserve(cloud->width * cloud->height);

  for (int v = 0; v < rgb.rows; ++v) {
    for (int u = 0; u < rgb.cols; ++u) {
      float Z = depth.at<float>(v, u);      // Asumimos que depth está en metros (CV_32FC1)
      if (Z <= 0.0f || std::isnan(Z)) {continue;}

      float X = (u - cx) * Z / fx;
      float Y = (v - cy) * Z / fy;

      cv::Vec3b color = rgb.at<cv::Vec3b>(v, u);
      pcl::PointXYZRGB point;
      point.x = X;
      point.y = Y;
      point.z = Z;
      point.r = color[2];
      point.g = color[1];
      point.b = color[0];

      cloud->points.push_back(point);
    }
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;  // para nube unidimensional
  return cloud;
}

cv::Mat project_point_cloud_2_image(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  const cv::Mat & K,
  int width,
  int height
)
{
  cv::Mat image(height, width, CV_8UC3, cv::Scalar(0, 0, 0));

  float fx = K.at<double>(0, 0);
  float fy = K.at<double>(1, 1);
  float cx = K.at<double>(0, 2);
  float cy = K.at<double>(1, 2);

  for (const auto & point : cloud->points) {
    if (point.z <= 0.0f) {continue;}

    int u = static_cast<int>((point.x * fx / point.z) + cx);
    int v = static_cast<int>((point.y * fy / point.z) + cy);

    if (u >= 0 && u < width && v >= 0 && v < height) {
      image.at<cv::Vec3b>(v, u) = cv::Vec3b(point.b, point.g, point.r);
    }
  }

  return image;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr filterColorHSV(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in,
  const cv::Scalar & hsv_min,
  const cv::Scalar & hsv_max
)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (const auto & point : cloud_in->points) {
      // Convertir RGB del punto a BGR (porque OpenCV usa BGR)
    cv::Mat bgr(1, 1, CV_8UC3, cv::Scalar(point.b, point.g, point.r));
    cv::Mat hsv;
    cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);

    cv::Vec3b hsv_pixel = hsv.at<cv::Vec3b>(0, 0);
    int h = hsv_pixel[0];
    int s = hsv_pixel[1];
    int v = hsv_pixel[2];

    if (h >= hsv_min[0] && h <= hsv_max[0] &&
      s >= hsv_min[1] && s <= hsv_max[1] &&
      v >= hsv_min[2] && v <= hsv_max[2])
    {
      cloud_filtered->points.push_back(point);
    }
  }

  cloud_filtered->width = cloud_filtered->points.size();
  cloud_filtered->height = 1;
  cloud_filtered->is_dense = false;

  return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr removeLargestPlane(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_in,
  double distance_threshold = 0.01   // en metros
)
{
  // Paso 1: crear objetos
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::ExtractIndices<pcl::PointXYZRGB> extract;

  // Paso 2: configurar segmentador
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(distance_threshold);
  seg.setInputCloud(cloud_in);

  // Paso 3: segmentar plano
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.empty()) {
    std::cerr << "No se encontró un plano." << std::endl;
    return cloud_in;   // no se elimina nada
  }

  // Paso 4: eliminar puntos del plano
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setNegative(true); // true = eliminar inliers
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
  extract.filter(*cloud_filtered);

  return cloud_filtered;
}


}

void initWindow()
{
  if (CVParams::running) {
    return;
  }

  CVParams::running = true;
  cv::namedWindow(CVParams::WINDOW_NAME);
  cv::createTrackbar(CVParams::OPTION, CVParams::WINDOW_NAME, nullptr, 6, 0);
  cv::setTrackbarPos(CVParams::OPTION, CVParams::WINDOW_NAME, 0);
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

  switch(CVParams::option) {
    case 0:
      processed_img = og_img;
      break;
    case 1:
      processed_img = CVFunctions::get_chessboard_img(og_img);
      break;
    case 2:
      processed_img = CVFunctions::draw_cube(og_img);
      break;
    case 3:
      processed_img = CVFunctions::get_disparity_map(left_rect, right_rect);
      break;
    case 4:
      processed_img = CVFunctions::project_point_cloud_2_image(
        CVFunctions::generate_point_cloud(rgb, depth, CVParams::camera_matrix),
        CVParams::camera_matrix,
        rgb.cols,
        rgb.rows
      );
      break;
    case 5:
      processed_img = CVFunctions::project_point_cloud_2_image(
        CVFunctions::filterColorHSV(
          CVFunctions::generate_point_cloud(rgb, depth,
        CVParams::camera_matrix),
          CVColors::HSV_PINK[0],
          CVColors::HSV_PINK[1]
        ),
        CVParams::camera_matrix,
        rgb.cols,
        rgb.rows
      );
      break;
    case 6:
      processed_img = CVFunctions::project_point_cloud_2_image(
        CVFunctions::removeLargestPlane(
          CVFunctions::generate_point_cloud(rgb, depth, CVParams::camera_matrix)
        ),
        CVParams::camera_matrix,
        rgb.cols,
        rgb.rows
      );
      break;
    default:
      break;
  }

  if (CVParams::option != 3) {
    cv::resize(processed_img, processed_img, cv::Size(), CVParams::img_scale,
        CVParams::img_scale);
  }
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
