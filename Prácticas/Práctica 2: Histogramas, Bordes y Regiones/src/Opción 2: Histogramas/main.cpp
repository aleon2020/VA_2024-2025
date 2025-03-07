#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

// Draw the intensity line for histograms
// We draw a line for each histogram
void 
drawHistogram(cv::Mat& histImage, cv::Mat hist, cv::Scalar color, int histSize, int bin_w, float alpha = 1.0) 
{
    for (int i = 1; i < histSize; i++) {
        line(
            histImage, cv::Point(bin_w * (i - 1), histImage.rows - cvRound(hist.at<float>(i - 1) * alpha)),
            cv::Point(bin_w * i, histImage.rows - cvRound(hist.at<float>(i) * alpha)),
            color, 2, 8, 0
        );
    }
}

int 
main(int argc, char **argv) 
{
    // Read image
    cv::CommandLineParser parser(argc, argv, "{@input | well_exposed.jpg | input image}");
    cv::Mat src = imread(cv::samples::findFile(parser.get<cv::String>("@input")), cv::IMREAD_COLOR);
    if (src.empty()) {
        return EXIT_FAILURE;
    }

    // Split BGR planes
    // The three planes of the color image are separated
    std::vector<cv::Mat> bgr_planes;
    cv::split(src, bgr_planes);

    // Establish the number of bins
    // The number of divisions (bins) is set,
    // which will be in the interval [0, 255]
    int histSize = 256;
    // Set the ranges ( for B,G,R) )
    float range[] = {0, 256};       //the upper boundary is exclusive
    const float* histRange = {range};
    // Indicates whether the bins have the same size (uniform)
    bool uniform = true, accumulate = false;

    // Compute the histograms for each channel
    // Histograms are calculated with the calcHist function
    cv::Mat b_hist, g_hist, r_hist;
    cv::calcHist(&bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &histSize, &histRange, uniform, accumulate);

    // Equalization
    // To equalize a channel, the equalizeHist function is used
    // which receives the channel in question, not the histogram
    cv::Mat b_eqhist, g_eqhist, r_eqhist;
    cv::equalizeHist(bgr_planes[0], b_eqhist);
    cv::equalizeHist(bgr_planes[1], g_eqhist);
    cv::equalizeHist(bgr_planes[2], r_eqhist);

    // If you want to display the equalized histogram,
    // do it as in the previous case

    // Compute the histograms for each channel
    cv::Mat b_histeq, g_histeq, r_histeq;
    cv::calcHist(&b_eqhist, 1, 0, cv::Mat(), b_histeq, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&g_eqhist, 1, 0, cv::Mat(), g_histeq, 1, &histSize, &histRange, uniform, accumulate);
    cv::calcHist(&r_eqhist, 1, 0, cv::Mat(), r_histeq, 1, &histSize, &histRange, uniform, accumulate);

    // Draw the histograms for B, G and R
    cv::Mat histImage(400, 512, CV_8UC3, cv::Scalar(0, 0, 0));
    int bin_w = cvRound((double)histImage.cols / histSize);

    // normalize the histograms between 0 and histImage.rows
    // Histograms are normalized between 0 and the number of rows in the calculated histogram
    cv::normalize(b_hist, b_hist, 0, histImage.rows, cv::NORM_MINMAX);
    cv::normalize(g_hist, g_hist, 0, histImage.rows, cv::NORM_MINMAX);
    cv::normalize(r_hist, r_hist, 0, histImage.rows, cv::NORM_MINMAX);

    // normalize the histograms between 0 and histImage.rows
    cv::normalize(b_histeq, b_histeq, 0, histImage.rows, cv::NORM_MINMAX);
    cv::normalize(g_histeq, g_histeq, 0, histImage.rows, cv::NORM_MINMAX);
    cv::normalize(r_histeq, r_histeq, 0, histImage.rows, cv::NORM_MINMAX);

    drawHistogram(histImage, b_hist, cv::Scalar(255, 0, 0), histSize, bin_w);
    drawHistogram(histImage, g_hist, cv::Scalar(0, 255, 0), histSize, bin_w);
    drawHistogram(histImage, r_hist, cv::Scalar(0, 0, 255), histSize, bin_w);
    drawHistogram(histImage, b_histeq, cv::Scalar(100, 0, 0), histSize, bin_w, 0.6);
    drawHistogram(histImage, g_histeq, cv::Scalar(0, 100, 0), histSize, bin_w, 0.6);
    drawHistogram(histImage, r_histeq, cv::Scalar(0, 0, 100), histSize, bin_w, 0.6);

    // Apply sequentially the 4 comparison methods between the histogram of the base image (hist_base) and the other histograms:
    // The 4 comparison methods are applied sequentially between the histograms of the base image (base_image) and the other histograms
    double corrB = cv::compareHist(b_hist, b_histeq, cv::HISTCMP_CORREL);
    double corrG = cv::compareHist(g_hist, g_histeq, cv::HISTCMP_CORREL);
    double corrR = cv::compareHist(r_hist, r_histeq, cv::HISTCMP_CORREL);

    // Write text in an image 
    cv::putText(histImage, "CompB: " + std::to_string(corrB), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0));
    cv::putText(histImage, "CompG: " + std::to_string(corrG), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0));
    cv::putText(histImage, "CompR: " + std::to_string(corrR), cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255));

    double mean_intensity = mean(src)[0];
    std::string exposure = (mean_intensity < 65) ? "Underexposed" : (mean_intensity > 190) ? "Overexposed" : "Well exposed";
    
    // Write text in an image 
    cv::putText(histImage, exposure, cv::Point(10, 80), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));

    // Create Equalized image
    // To see the equalized image, it is composed from the
    // three channels resulting from the equalization
    std::vector<cv::Mat> equalized = {b_eqhist, g_eqhist, r_eqhist};
    // Merge channels
    cv::Mat equalized_image;
    cv::merge(equalized, equalized_image);

    // Show images
    // The equalized image and its histogram are displayed
    cv::imshow("Original Image", src);
    cv::imshow("Equalized Image", equalized_image);
    cv::imshow("OPTION 2", histImage);

    cv::waitKey();
    return EXIT_SUCCESS;
}

