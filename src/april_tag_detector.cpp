#include "april_tag_detector.hpp"

// 引入全局日志模块
#include "logger.hpp"

const cv::Scalar AprilTagDetector::kColorPink(255, 0, 255);
const cv::Scalar AprilTagDetector::kColorRed(0, 0, 255);
const cv::Scalar AprilTagDetector::kColorGreen(0, 255, 0);

AprilTagDetector::AprilTagDetector()
{
    tf = tag25h9_create();
    td = apriltag_detector_create();
    if (!td || !tf)
    {
        logging::get_logger()->error("Failed to create AprilTag detector");
        throw std::runtime_error("Failed to create AprilTag detector");
    }
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 2.0;
    td->nthreads = 4;
    td->debug = 0;
    td->refine_edges = 1;
}

AprilTagDetector::~AprilTagDetector()
{
    if (td)
        apriltag_detector_destroy(td);
    if (tf)
        tag25h9_destroy(tf);
}

DetectionResult AprilTagDetector::detect(cv::Mat &frame, bool drawOverlay)
{
    DetectionResult result;

    try
    {
        if (frame.empty())
        {
            logging::get_logger()->error("Input frame is empty");
            throw std::runtime_error("Input frame is empty");
        }

        result.width = frame.cols;
        result.height = frame.rows;

        // 图像预处理
        cv::Mat gray, binary;
        if (frame.channels() == 3)
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        else
            gray = frame;

        cv::threshold(gray, binary, 110, 255, cv::THRESH_BINARY);

        image_u8_t *im = image_u8_create(binary.cols, binary.rows);
        if (!im)
        {
            logging::get_logger()->error("Failed to allocate image_u8 buffer");
            throw std::bad_alloc();
        }

        for (int y = 0; y < binary.rows; ++y)
        {
            memcpy(im->buf + y * im->stride,
                   binary.data + y * binary.step,
                   binary.cols);
        }

        zarray_t *detections = apriltag_detector_detect(td, im);
        if (!detections)
        {
            image_u8_destroy(im);
            return result;
        }

        bool hasDetections = false;

        for (int i = 0; i < zarray_size(detections); ++i)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            hasDetections = true;

            const cv::Point2f points[4] = {
                {det->p[3][0], det->p[3][1]}, // 左上
                {det->p[0][0], det->p[0][1]}, // 右上
                {det->p[1][0], det->p[1][1]}, // 右下
                {det->p[2][0], det->p[2][1]}  // 左下
            };

            const cv::Point2f center(det->c[0], det->c[1]);
            result.centers.push_back(center);

            if (drawOverlay)
            {
                cv::line(frame, points[0], points[1], kColorPink, 2, cv::LINE_AA);
                cv::line(frame, points[1], points[2], kColorPink, 2, cv::LINE_AA);
                cv::line(frame, points[2], points[3], kColorPink, 2, cv::LINE_AA);
                cv::line(frame, points[3], points[0], kColorPink, 2, cv::LINE_AA);
                cv::circle(frame, center, 5, kColorRed, -1);
                cv::putText(frame, "tag25h9",
                            cv::Point(points[0].x, points[0].y - 15),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5,
                            kColorGreen, 2);
            }
        }

        apriltag_detections_destroy(detections);
        result.detected = hasDetections;
        image_u8_destroy(im);
    }
    catch (const cv::Exception &e)
    {
        logging::get_logger()->error("OpenCV exception: {}", e.what());
        result.centers.clear();
        result.width = result.height = 0;
    }
    catch (...)
    {
        logging::get_logger()->error("Unknown exception occurred");
        result.centers.clear();
        result.width = result.height = 0;
    }

    return result;
}