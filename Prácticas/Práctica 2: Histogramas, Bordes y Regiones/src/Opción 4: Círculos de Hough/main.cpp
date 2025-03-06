#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

int 
main(int argc, char **argv)
{
    const char *filename = argc >= 2 ? argv[1] : "balls.jpg";
    
    // Load an image
    cv::Mat src = imread(cv::samples::findFile(filename), cv::IMREAD_COLOR);
    
    // Check if image is loaded fine
    if (src.empty()) {
        printf("Error opening image\n");
        return EXIT_FAILURE;
    }

    cv::Mat hsv, mask, pinkOnly, gray;
    
    // Convert to HSV color space
    cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV);
    
    // Define range for pink color and create mask
    cv::Scalar lower_pink = cv::Scalar(140, 50, 50);
    cv::Scalar upper_pink = cv::Scalar(170, 255, 255);
    cv::inRange(hsv, lower_pink, upper_pink, mask);
    
    // Extract only pink areas
    cv::bitwise_and(src, src, pinkOnly, mask);
    
    // Convert to grayscale and blur for Hough Transform
    cv::cvtColor(pinkOnly, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 5);
    
    // Detect circles using Hough Transform
    std::vector<cv::Vec3f> circles;
    HoughCircles(
        gray, circles, cv::HOUGH_GRADIENT, 1,
        gray.rows / 16,                 // change this value to detect circles with different distances to each other
        100, 30, 1, 30                  // change the last two parameters (min_radius & max_radius) to detect larger circles
    );
    
    for (size_t i = 0; i < circles.size(); i++) {
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0], c[1]);
        // circle center
        cv::circle(src, center, 1, cv::Scalar(0, 255, 255), 3, cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle(src, center, radius, cv::Scalar(255, 0, 0), 3, cv::LINE_AA);
    }
    
    // Show image and circles
    cv::imshow("OPTION 4", src);
    cv::waitKey();
    return EXIT_SUCCESS;
}
