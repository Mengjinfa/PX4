#include "apriltag_tracker.hpp"
#include "sim_camera_module.hpp"
#include <opencv2/opencv.hpp>
#include <zlib.h>

// Gazebo仿真环境中相机图像主题
std::string subscribePtr = "/gazebo/default/iris/base_link/camera/image";

// 获取最新帧函数（仿真模式专用）
cv::Mat get_latest_frame()
{
    cv::Mat frame = GazeboCamera::Instance()->GetNextFrame(); // 从Gazebo仿真相机获取最新图像帧
    return frame;
}

// 初始化跟踪器状态和参数
AprilTagTracker::AprilTagTracker() : running_(false), tf_(nullptr), td_(nullptr), decimate_(1.0), blur_(0.0), threads_(4), refine_edges_(true)
{
    // 创建AprilTag检测器
    td_ = apriltag_detector_create();       // 创建AprilTag检测器实例
    tf_ = tag25h9_create();                 // 创建tag25h9标签家族
    apriltag_detector_add_family(td_, tf_); // 将标签家族添加到检测器

    // 配置检测器参数
    td_->quad_decimate = decimate_;    // 降低图像分辨率以加速检测
    td_->quad_sigma = blur_;           // 应用高斯模糊以减少噪点
    td_->nthreads = threads_;          // 设置处理线程数
    td_->debug = 0;                    // 禁用调试模式
    td_->refine_edges = refine_edges_; // 启用边缘优化以提高精度

    // 启用OpenCV优化并设置线程数
    cv::setUseOptimized(true);
    cv::setNumThreads(threads_);
}

// 析构函数
AprilTagTracker::~AprilTagTracker()
{
    stop(); // 停止处理线程

    // 释放AprilTag资源（先释放标签家族，再释放检测器）
    if (tf_)
        tag25h9_destroy(tf_);
    if (td_)
        apriltag_detector_destroy(td_);

    cv::destroyAllWindows(); // 关闭所有OpenCV窗口
}

// 启动跟踪器
void AprilTagTracker::start(int argc, char *argv[])
{
    GazeboCamera::Instance()->init(argc, argv, subscribePtr); // 初始化Gazebo仿真相机
    GazeboCamera::Instance()->start();                        // 启动Gazebo仿真相机模块

    if (running_)
        return; // 避免重复启动
    running_ = true;
    processing_thread_ = std::thread(&AprilTagTracker::processingLoop, this);
    std::cout << "跟踪器已启动，使用仿真模式" << std::endl;
}

// 停止跟踪器
void AprilTagTracker::stop()
{
    if (!running_)
        return; // 避免重复停止
    running_ = false;
    if (processing_thread_.joinable())
        processing_thread_.join(); // 等待线程结束
    std::cout << "跟踪器已停止" << std::endl;
}

// 获取最新检测结果
AprilTagData AprilTagTracker::getData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_); // 加锁保护数据
    return latest_data_;
}

// 设置图像降采样比例
void AprilTagTracker::setDecimate(double decimate)
{
    decimate_ = decimate;
    if (td_)
        td_->quad_decimate = decimate_;
}

// 设置高斯模糊参数
void AprilTagTracker::setBlur(double blur)
{
    blur_ = blur;
    if (td_)
        td_->quad_sigma = blur_;
}

// 设置处理线程数
void AprilTagTracker::setThreads(int threads)
{
    threads_ = threads;
    if (td_)
        td_->nthreads = threads_;
    cv::setNumThreads(threads_);
}

// 设置边缘优化参数
void AprilTagTracker::setRefineEdges(bool refine)
{
    refine_edges_ = refine;
    if (td_)
        td_->refine_edges = refine_edges_;
}

// 处理循环函数
void AprilTagTracker::processingLoop()
{
    const int frame_rate = 30;
    cv::Mat frame;

    while (running_)
    {
        frame = get_latest_frame(); // 从仿真环境获取帧

        if (frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frame_rate));
            continue;
        }

        AprilTagData result = processFrame(frame); // 处理当前帧

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            latest_data_ = result; // 更新最新检测结果
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frame_rate));
    }
}

// 图像预处理函数
cv::Mat AprilTagTracker::preprocessImage(const cv::Mat &frame) const
{
    cv::Mat ycrcb, processed;
    cv::cvtColor(frame, ycrcb, cv::COLOR_BGR2YCrCb); // 转换到YCrCb色彩空间

    // 分离通道，对亮度通道(Y)进行增强
    std::vector<cv::Mat> channels;
    cv::split(ycrcb, channels);

    // 应用对比度受限的自适应直方图均衡化(CLAHE)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(channels[0], channels[0]); // 仅处理亮度通道

    cv::merge(channels, ycrcb);                          // 合并通道
    cv::cvtColor(ycrcb, processed, cv::COLOR_YCrCb2BGR); // 转回BGR色彩空间

    // 应用高斯模糊(如果blur_参数大于0)
    if (blur_ > 0)
    {
        cv::GaussianBlur(processed, processed, cv::Size(0, 0), blur_);
    }
    return processed;
}

// 计算标签面积函数
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

// 处理单帧图像函数
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

        if (best_det)
        {
            // 提取标签中心坐标
            result.x = static_cast<int>(best_det->c[0]); // 列坐标 -> x
            result.y = static_cast<int>(best_det->c[1]); // 行坐标 -> y
            result.iffind = true;                        // 标记找到标签
            result.id = best_det->id;                    // 标签ID

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

    // 显示处理结果（可选，如需可视化请取消注释）
    cv::imshow("AprilTag Detection", display);
    cv::waitKey(1); // 等待1ms，允许窗口响应事件

    return result;
}