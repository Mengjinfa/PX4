#include "april_tag_detector.hpp"
#include "logger.hpp"

// 定义绘制AprilTag轮廓和中心点的颜色常量
const cv::Scalar AprilTagDetector::kColorPink(255, 0, 255);
const cv::Scalar AprilTagDetector::kColorRed(0, 0, 255);
const cv::Scalar AprilTagDetector::kColorGreen(0, 255, 0);

/**
 * 构造函数：初始化AprilTag检测器
 */
AprilTagDetector::AprilTagDetector()
{
    // 创建AprilTag家族和检测器实例
    tf = tag25h9_create();
    td = apriltag_detector_create();
    if (!td || !tf)
    {
        logging::get_logger()->error("Failed to create AprilTag detector");
        throw std::runtime_error("Failed to create AprilTag detector");
    }

    // 将tag25h9家族添加到检测器
    apriltag_detector_add_family(td, tf);

    // 配置检测器参数
    td->quad_decimate = 2.0; // 图像降采样因子，提高检测速度
    td->nthreads = 4;        // 使用4个线程并行处理
    td->debug = 0;           // 禁用调试模式
    td->refine_edges = 1;    // 启用边缘精修以提高精度
}

/**
 * 析构函数：释放AprilTag检测器资源
 */
AprilTagDetector::~AprilTagDetector()
{
    if (td)
        apriltag_detector_destroy(td);
    if (tf)
        tag25h9_destroy(tf);
}

/**
 * 在图像中检测AprilTag并返回结果
 * @param frame 输入图像帧
 * @param drawOverlay 是否在原图上绘制检测结果
 * @return 包含检测结果的结构体
 */
DetectionResult AprilTagDetector::detect(cv::Mat &frame, bool drawOverlay)
{
    DetectionResult result;

    try
    {
        // 检查输入帧是否为空
        if (frame.empty())
        {
            logging::get_logger()->error("Input frame is empty");
            throw std::runtime_error("Input frame is empty");
        }

        // 保存图像尺寸到结果结构体
        result.width = frame.cols;
        result.height = frame.rows;

        // 图像预处理：转换为灰度图并进行二值化
        cv::Mat gray, binary;
        if (frame.channels() == 3)
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        else
            gray = frame;

        // 使用固定阈值进行二值化处理
        cv::threshold(gray, binary, 110, 255, cv::THRESH_BINARY);

        // 创建AprilTag库使用的图像结构
        image_u8_t *im = image_u8_create(binary.cols, binary.rows);
        if (!im)
        {
            logging::get_logger()->error("Failed to allocate image_u8 buffer");
            throw std::bad_alloc();
        }

        // 将OpenCV图像数据复制到AprilTag库的图像结构中
        for (int y = 0; y < binary.rows; ++y)
        {
            memcpy(im->buf + y * im->stride, binary.data + y * binary.step, binary.cols);
        }

        // 执行AprilTag检测
        zarray_t *detections = apriltag_detector_detect(td, im);
        if (!detections)
        {
            image_u8_destroy(im);
            return result;
        }

        bool hasDetections = false;

        // 处理检测到的每个AprilTag
        for (int i = 0; i < zarray_size(detections); ++i)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);
            hasDetections = true;

            // 提取AprilTag四个角点和中心点坐标
            const cv::Point2f points[4] = {
                {det->p[3][0], det->p[3][1]}, // 左上
                {det->p[0][0], det->p[0][1]}, // 右上
                {det->p[1][0], det->p[1][1]}, // 右下
                {det->p[2][0], det->p[2][1]}  // 左下
            };

            const cv::Point2f center(det->c[0], det->c[1]);
            result.centers.push_back(center);

            // 如果需要绘制检测结果，则在原图上绘制轮廓和中心点
            if (drawOverlay)
            {
                cv::line(frame, points[0], points[1], kColorPink, 2, cv::LINE_AA);
                cv::line(frame, points[1], points[2], kColorPink, 2, cv::LINE_AA);
                cv::line(frame, points[2], points[3], kColorPink, 2, cv::LINE_AA);
                cv::line(frame, points[3], points[0], kColorPink, 2, cv::LINE_AA);
                cv::circle(frame, center, 5, kColorRed, -1);
            }
        }

        // 释放检测结果和图像资源
        apriltag_detections_destroy(detections);
        result.detected = hasDetections;
        image_u8_destroy(im);
    }
    catch (const cv::Exception &e)
    {
        // 处理OpenCV异常
        logging::get_logger()->error("OpenCV exception: {}", e.what());
        result.centers.clear();
        result.width = result.height = 0;
    }
    catch (...)
    {
        // 处理未知异常
        logging::get_logger()->error("Unknown exception occurred");
        result.centers.clear();
        result.width = result.height = 0;
    }

    return result;
}