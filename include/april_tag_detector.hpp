#pragma once
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag25h9.h>

struct DetectionResult {
    bool detected = false;
    std::vector<cv::Point2f> centers;
    int width = 0, height = 0;
};

class AprilTagDetector {
public:
    AprilTagDetector();
    ~AprilTagDetector();

    DetectionResult detect(cv::Mat& frame, bool drawOverlay = true);

private:
    apriltag_family_t* tf;
    apriltag_detector_t* td;

    static const cv::Scalar kColorPink;
    static const cv::Scalar kColorRed;
    static const cv::Scalar kColorGreen;
};