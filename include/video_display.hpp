#pragma once
#include <opencv2/opencv.hpp>
#include <atomic>

class VideoDisplay {
public:
    static void showFrame(const cv::Mat& frame);
    static bool shouldExit();
};