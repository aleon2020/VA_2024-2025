#include <opencv2/opencv.hpp>
#include <iostream>

cv::Mat originalImage, transformedImage;
std::vector<cv::Point2f> points;
bool transformed = false;

// on_mouse() function
// Create mouse callback
void 
on_mouse(int event, int x, int y, int, void*) 
{
    if (event == cv::EVENT_LBUTTONDOWN) {

        if (transformed) {
            transformedImage = originalImage.clone();
            points.clear();
            transformed = false;
            cv::imshow("OPTION 1", transformedImage);
            return;
        }

        if (points.size() < 4) {
            points.emplace_back(static_cast<float>(x), static_cast<float>(y));
            cv::putText(transformedImage, std::to_string(points.size()), cv::Point(x, y),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
            cv::imshow("OPTION 1", transformedImage);
        }
        
        if (points.size() == 4) {
            std::vector<cv::Point2f> destPoints = {
                {0.0f, 0.0f},
                {static_cast<float>(originalImage.cols - 1), 0.0f},
                {0.0f, static_cast<float>(originalImage.rows - 1)},
                {static_cast<float>(originalImage.cols - 1), static_cast<float>(originalImage.rows - 1)}
            };
            cv::Mat perspectiveMatrix = cv::getPerspectiveTransform(points, destPoints);
            cv::warpPerspective(originalImage, transformedImage, perspectiveMatrix, originalImage.size());
            cv::imshow("OPTION 1", transformedImage);
            transformed = true;
        }
    }
}

// main() function
// Main function of the program
int 
main() 
{
    // Read image
    originalImage = cv::imread("clics.jpg", cv::IMREAD_COLOR);

    // Check if image is loaded fine
    if (originalImage.empty()) {
        std::cerr << "Error: No se pudo cargar la imagen." << std::endl;
        return -1;
    }
    
    transformedImage = originalImage.clone();

    // Show results
    cv::namedWindow("OPTION 1");

    // Create mouse callback
    cv::setMouseCallback("OPTION 1", on_mouse);

    // Show results
    cv::imshow("OPTION 1", transformedImage);

    // Wait to press a key
    cv::waitKey(0);
    return 0;
}
