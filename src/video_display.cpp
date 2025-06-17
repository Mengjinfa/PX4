#include "video_display.hpp"

void VideoDisplay::showFrame(const cv::Mat& frame) {
    cv::imshow("Tracking", frame);
}

bool VideoDisplay::shouldExit() {
    return cv::waitKey(1) == 27;
}