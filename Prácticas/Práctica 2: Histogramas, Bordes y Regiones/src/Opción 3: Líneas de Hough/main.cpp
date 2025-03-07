#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

// Declare the output variables
cv::Mat src, dst, colorSrc;
int accumulatorThreshold = 200;

// drawHoughLines() function
// Calculate the linear Hough transform
void 
drawHoughLines(int, void*) 
{
    cv::Mat cdst;

    // Edge detection
    cv::Canny(src, dst, 50, 200, 3);

    // Copy edges to the images that will display the results in BGR
    cv::cvtColor(src, colorSrc, cv::COLOR_GRAY2BGR);
    
    // Standard Hough Line Transform
    std::vector<cv::Vec2f> lines;   // will hold the results of the detection (rho, theta)
    cv::HoughLines(dst, lines, 1, CV_PI / 180, accumulatorThreshold, 0, 0);   // runs the actual detection
    
    // Draw the lines
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        cv::line(colorSrc, pt1, pt2, cv::Scalar(0, 0, 255), 2, cv::LINE_AA);
    }
    
    // Write text in an image 
    std::string text = "N: " + std::to_string(lines.size());
    cv::putText(colorSrc, text, cv::Point(10, 20), 
            cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));
    
    // Show results
    cv::imshow("OPTION 3", colorSrc);
}

int 
main(int argc, char **argv) 
{
    // Declare the output variables
    const char *default_file = "lines.jpg";
    const char *filename = argc >= 2 ? argv[1] : default_file;

    // Load an image
    src = cv::imread(cv::samples::findFile(filename), cv::IMREAD_GRAYSCALE);
    
    // Check if image is loaded fine
    if (src.empty()) {
        printf("Error opening image\n");
        return -1;
    }
    
    // Show results
    cv::namedWindow("OPTION 3");
    cv::createTrackbar("Hough accumulator [0-255]", "OPTION 3", &accumulatorThreshold, 255, drawHoughLines);
    drawHoughLines(0, 0);
    
    // Wait and Exit
    cv::waitKey();
    return 0;
}
