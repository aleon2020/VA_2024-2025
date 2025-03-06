#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;
using namespace std;

Mat src, dst, colorSrc;
int accumulatorThreshold = 200;

void onTrackbar(int, void*) {
    Mat cdst;
    Canny(src, dst, 50, 200, 3);
    cvtColor(src, colorSrc, COLOR_GRAY2BGR);
    
    vector<Vec2f> lines;
    HoughLines(dst, lines, 1, CV_PI / 180, accumulatorThreshold, 0, 0);
    
    for (size_t i = 0; i < lines.size(); i++) {
        float rho = lines[i][0], theta = lines[i][1];
        Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a * rho, y0 = b * rho;
        pt1.x = cvRound(x0 + 1000 * (-b));
        pt1.y = cvRound(y0 + 1000 * (a));
        pt2.x = cvRound(x0 - 1000 * (-b));
        pt2.y = cvRound(y0 - 1000 * (a));
        line(colorSrc, pt1, pt2, Scalar(0, 0, 255), 2, LINE_AA);
    }
    
    string text = "N: " + to_string(lines.size());
    putText(colorSrc, text, Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255));
    
    imshow("OPTION 3", colorSrc);
}

int main(int argc, char **argv) {
    const char *default_file = "lines.jpg";
    const char *filename = argc >= 2 ? argv[1] : default_file;
    src = imread(samples::findFile(filename), IMREAD_GRAYSCALE);
    
    if (src.empty()) {
        printf("Error opening image\n");
        return -1;
    }
    
    namedWindow("OPTION 3");
    createTrackbar("Hough accumulator [0-255]", "OPTION 3", &accumulatorThreshold, 255, onTrackbar);
    onTrackbar(0, 0);
    
    waitKey();
    return 0;
}

