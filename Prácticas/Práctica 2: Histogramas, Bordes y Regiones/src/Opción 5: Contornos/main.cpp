#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

cv::Mat image;
int areaThreshold = 1000;

void 
on_trackbar(int, void*) 
{
    // Create image variables
    cv::Mat gray, edges, gauss, result;
    int rectCount = 0;

    // Convert image to grayscale
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

    // Gaussian blur
    cv::GaussianBlur(gray, gauss, cv::Size(5, 5), 0);

    // Image processing
    cv::Canny(gauss, edges, 50, 100, 3);

    // Contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(edges, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    result = image.clone();

    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > areaThreshold) {
            std::vector<cv::Point> approx;
            cv::approxPolyDP(contours[i], approx, arcLength(contours[i], true) * 0.02, true);
            
            if (approx.size() == 4 && cv::isContourConvex(approx)) {

                // Drawing contours
                // iterate through all the top-level contours,
                // draw each connected component with its own random color
                // If for the contour i there are no next, previous, parent, or nested contours, the corresponding elements of hierarchy[i] will be negative.
                cv::Scalar color(rand() & 255, rand() & 255, rand() & 255);
                cv::drawContours(result, contours, (int)i, color, 2);
                
                // Calculate moments
                cv::Moments m = cv::moments(contours[i]);
                if (m.m00 != 0) {
                    int cx = (int)(m.m10 / m.m00);
                    int cy = (int)(m.m01 / m.m00);
                    cv::circle(result, cv::Point(cx, cy), 5, color, -1);
                    cv::putText(result, std::to_string((int)area), cv::Point(cx + 10, cy + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, color);
                }
                rectCount++;
            }
        }
    }

    // Write text in an image
    cv::putText(result, "Rectangles: " + std::to_string(rectCount), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
    
    // Show results
    cv::imshow("OPTION 5", result);
}

int 
main() 
{
    // Read image
    image = cv::imread("contours.jpg", cv::IMREAD_COLOR);

    // Check if image is loaded fine
    if (image.empty()) {
        std::cout << "Error: No se pudo cargar la imagen!" << std::endl;
        return -1;
    }

    // Show results
    cv::namedWindow("OPTION 5", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("Area [0-20k]", "OPTION 5", &areaThreshold, 20000, on_trackbar);
    on_trackbar(0, 0);
    
    // Wait to press a key
    cv::waitKey(0);
    return 0;
}