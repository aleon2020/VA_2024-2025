#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    const char *filename = argc >= 2 ? argv[1] : "smarties.png";
    
    // Load an image
    Mat src = imread(samples::findFile(filename), IMREAD_COLOR);
    
    if (src.empty()) {
        printf("Error opening image\n");
        return EXIT_FAILURE;
    }

    Mat hsv, mask, pinkOnly, gray;
    
    // Convert to HSV color space
    cvtColor(src, hsv, COLOR_BGR2HSV);
    
    // Define range for pink color and create mask
    Scalar lower_pink = Scalar(140, 50, 50);
    Scalar upper_pink = Scalar(170, 255, 255);
    inRange(hsv, lower_pink, upper_pink, mask);
    
    // Extract only pink areas
    bitwise_and(src, src, pinkOnly, mask);
    
    // Convert to grayscale and blur for Hough Transform
    cvtColor(pinkOnly, gray, COLOR_BGR2GRAY);
    medianBlur(gray, gray, 5);
    
    // Detect circles using Hough Transform
    vector<Vec3f> circles;
    HoughCircles(
        gray, circles, HOUGH_GRADIENT, 1, gray.rows / 16,
        100, 30, 1, 100);
    
    for (size_t i = 0; i < circles.size(); i++) {
        Vec3i c = circles[i];
        Point center = Point(c[0], c[1]);
        int radius = c[2];
        
        // Draw center in yellow
        circle(src, center, 3, Scalar(0, 255, 255), -1, LINE_AA);
        
        // Draw circle outline in blue
        circle(src, center, radius, Scalar(255, 0, 0), 3, LINE_AA);
    }
    
    // Show result
    imshow("OPTION 4", src);
    waitKey();
    return EXIT_SUCCESS;
}
