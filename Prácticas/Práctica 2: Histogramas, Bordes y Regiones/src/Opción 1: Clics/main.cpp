#include <opencv2/opencv.hpp>
#include <iostream>

cv::Mat originalImage, transformedImage;
std::vector<cv::Point2f> points;
bool transformed = false;

void 
onMouse(int event, int x, int y, int, void*) 
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

int 
main() 
{
    originalImage = cv::imread("clics.jpg", cv::IMREAD_COLOR);

    if (originalImage.empty()) {
        std::cerr << "Error: No se pudo cargar la imagen." << std::endl;
        return -1;
    }
    
    transformedImage = originalImage.clone();
    cv::namedWindow("OPTION 1");
    cv::setMouseCallback("OPTION 1", onMouse);
    cv::imshow("OPTION 1", transformedImage);
    cv::waitKey(0);
    return 0;
}
