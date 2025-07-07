#include "landing_function.hpp"

#include "flight_procedure.hpp"
#include "landing_state_machine.hpp"
#include "telemetry_monitor.hpp"

#include <cmath>

// // 构造函数实现
// AprilTagTracker::AprilTagTracker() : td_(nullptr), tf_(nullptr), running_(false), decimate_(1.0), blur_(0.2), threads_(4), refine_edges_(true)
// {
//     // 创建AprilTag检测器
//     td_ = apriltag_detector_create();       // 创建AprilTag检测器实例
//     tf_ = tag25h9_create();                 // 创建tag25h9标签家族
//     apriltag_detector_add_family(td_, tf_); // 将标签家族添加到检测器

//     // 配置检测器参数
//     td_->quad_decimate = decimate_;    // 降低图像分辨率以加速检测
//     td_->quad_sigma = blur_;           // 应用高斯模糊以减少噪点
//     td_->nthreads = threads_;          // 设置处理线程数
//     td_->debug = 0;                    // 禁用调试模式
//     td_->refine_edges = refine_edges_; // 启用边缘优化以提高精度

//     // 启用OpenCV优化并设置线程数
//     cv::setUseOptimized(true);
//     cv::setNumThreads(threads_); // 设置OpenCV的线程数以提高性能
// }

// // 析构函数实现
// AprilTagTracker::~AprilTagTracker()
// {
//     // 释放AprilTag资源（先释放标签家族，再释放检测器）
//     if (tf_)
//         tag25h9_destroy(tf_);
//     if (td_)
//         apriltag_detector_destroy(td_);

//     cv::destroyAllWindows(); // 关闭所有OpenCV窗口
// }

// 图像预处理函数实现
cv::Mat AprilTagTracker::preprocessImage(const cv::Mat &frame) const
{
    if (frame.empty())
    {
        std::cerr << "错误：在 preprocessImage 中为空帧" << std::endl;
        return cv::Mat();
    }

    std::cout << "图像通道数：" << frame.channels() << "，尺寸：" << frame.rows << "x" << frame.cols << std::endl;

    cv::Mat processed;
    try
    {
        // 根据输入图像通道数选择合适的转换方式
        if (frame.channels() == 3)
        {
            cv::cvtColor(frame, processed, cv::COLOR_BGR2GRAY); // 3通道BGR转灰度
        }
        else if (frame.channels() == 4)
        {
            cv::cvtColor(frame, processed, cv::COLOR_RGBA2GRAY); // 4通道RGBA转灰度
        }
        else if (frame.channels() == 1)
        {
            processed = frame.clone(); // 已经是灰度图，直接复制
        }
        else
        {
            std::cerr << "不支持的通道数：" << frame.channels() << std::endl;
            return frame.clone();
        }
    }
    catch (const cv::Exception &e)
    {
        std::cerr << "颜色转换错误：" << e.what() << std::endl;
        return frame.clone();
    }

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(processed, processed);

    // 应用高斯模糊（如果 blur_ 参数大于 0）
    if (blur_ > 0)
    {
        cv::GaussianBlur(processed, processed, cv::Size(0, 0), blur_);
    }

    return processed;
}

// 计算标签面积函数实现
double AprilTagTracker::calculateTagArea(const apriltag_detection_t *det) const
{
    // 提取四个顶点坐标
    double x1 = det->p[0][0], y1 = det->p[0][1];
    double x2 = det->p[1][0], y2 = det->p[1][1];
    double x3 = det->p[2][0], y3 = det->p[2][1];
    double x4 = det->p[3][0], y4 = det->p[3][1];

    // 使用向量叉积计算面积
    double area = 0.5 * abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1) + (x3 - x2) * (y4 - y2) - (x4 - x2) * (y3 - y2));

    return area;
}

// 处理单帧图像函数实现
AprilTagData AprilTagTracker::processFrame(const cv::Mat &frame) const
{
    // 初始化结果结构体(默认未找到标签)
    AprilTagData result = {false, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0};

    cv::Mat processed = preprocessImage(frame);        // 图像预处理
    cv::Mat display = processed.clone();               // 用于显示的图像副本
    cv::Mat gray;                                      //
    cv::cvtColor(processed, gray, cv::COLOR_BGR2GRAY); // 转换为灰度图像

    // 准备AprilTag检测所需的图像结构
    image_u8_t im = {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data};

    // 执行AprilTag检测
    zarray_t *detections = apriltag_detector_detect(td_, &im);

    if (zarray_size(detections) > 0)
    {
        apriltag_detection_t *best_det = nullptr;
        double min_area = std::numeric_limits<double>::max(); // 最小面积初始化

        // 遍历所有检测结果，寻找面积最小的标签
        for (int i = 0; i < zarray_size(detections); i++)
        {
            apriltag_detection_t *det;
            zarray_get(detections, i, &det);

            // 计算标签面积
            double area = calculateTagArea(det);

            // 更新最小面积标签
            if (area < min_area)
            {
                min_area = area;
                best_det = det;
            }
        }

        // 如果找到最小面积的标签
        if (best_det)
        {
            // 提取标签中心坐标
            result.x = static_cast<int>(best_det->c[0]); // 列坐标 -> x
            result.y = static_cast<int>(best_det->c[1]); // 行坐标 -> y
            result.iffind = true;                        // 标记找到标签

            // 获取画面大小
            result.width = frame.cols;
            result.height = frame.rows;

            // 计算与图像中心的偏差
            result.err_x = (result.height / 2.0) - result.y; // X方向偏差
            result.err_y = (result.width / 2.0) - result.x;  // Y方向偏差

            // 相对于图像全宽 / 全高归一化（结果范围 [-0.5, 0.5]）：
            result.norm_err_x = result.err_x / result.width;
            result.norm_err_y = result.err_y / result.height;

            // 绘制标签边界
            for (int i = 0; i < 4; i++)
            {
                cv::Point pt1(best_det->p[i][0], best_det->p[i][1]);
                cv::Point pt2(best_det->p[(i + 1) % 4][0], best_det->p[(i + 1) % 4][1]);
                cv::line(display, pt1, pt2, cv::Scalar(0, 255, 0), 2); // 绿色边界线
            }
            // 绘制标签中心
            cv::circle(display, cv::Point(result.x, result.y), 5, cv::Scalar(0, 0, 255), -1); // 红色中心点
        }
    }

    // 释放检测结果
    apriltag_detections_destroy(detections);

    // 显示处理结果
    cv::imshow("AprilTag Detection", display);
    cv::waitKey(1);

    return result;
}

// 跟踪器主循环函数实现
AprilTagData AprilTagTracker::process()
{
    cv::Mat frame = get_latest_frame(); // 获取Gazebo相机最新帧

    if (frame.empty())
    {
        return AprilTagData{false, 0, 0, 0, 0, 0, 0.0, 0.0, 0.0, 0.0}; // 没有画面时返回默认值
    }

    return processFrame(frame); // 处理单帧图像并返回结果
}
