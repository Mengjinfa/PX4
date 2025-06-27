#include "apriltag_tracker.hpp"
#include "sim_camera_module.hpp"

// Gazebo仿真环境中相机图像主题
std::string subscribePtr = "/gazebo/default/iris/base_link/camera/image";

// 获取最新帧函数（仿真模式专用）
cv::Mat get_latest_frame()
{
    return GazeboCamera::Instance()->GetNextFrame(); // 从Gazebo仿真相机获取最新图像帧
}

// 构造函数实现
AprilTagTracker::AprilTagTracker() : td_(nullptr), tf_(nullptr), running_(false),
                                     decimate_(1.0), blur_(0.2), threads_(4), refine_edges_(true)
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
    cv::setNumThreads(threads_); // 设置OpenCV的线程数以提高性能
}

// 析构函数实现
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

// 启动跟踪器实现
void AprilTagTracker::start(int argc, char *argv[])
{
    // 检查是否已启动
    if (running_)
    {
        std::cout << "AprilTag跟踪器已在运行中" << std::endl;
        return;
    }

    // 初始化并启动Gazebo相机
    GazeboCamera::Instance()->init(argc, argv, subscribePtr);
    GazeboCamera::Instance()->start();

    // 标记为运行中并启动处理线程
    running_ = true;
    processing_thread_ = std::thread(&AprilTagTracker::processingLoop, this);
    std::cout << "AprilTag跟踪器已启动,单线程处理" << std::endl;
}

// 停止跟踪器实现
void AprilTagTracker::stop()
{
    if (!running_)
        return; // 避免重复停止

    // 标记为停止并等待线程结束
    running_ = false;
    if (processing_thread_.joinable())
        processing_thread_.join(); // 等待线程结束
    std::cout << "AprilTag跟踪器已停止" << std::endl;
}

// 图像预处理函数实现
cv::Mat AprilTagTracker::preprocessImage(const cv::Mat &frame) const
{
    cv::Mat processed;

    // 如果颜色信息对 AprilTag 检测不关键，则立即转换为灰度图。
    // AprilTag 检测主要依赖于边缘特征，这些特征在灰度图中也能很好地保留。
    // 这减少了后续步骤的数据大小和处理复杂性。
    cv::cvtColor(frame, processed, cv::COLOR_BGR2GRAY);

    // 直接对灰度图像应用 CLAHE。
    // 这样可以增强局部对比度，而无需来回转换到 YCrCb，从而简化了处理流程。
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(3.0, cv::Size(8, 8));
    clahe->apply(processed, processed);

    // 应用高斯模糊（如果 blur_ 参数大于 0）
    if (blur_ > 0)
    {
        cv::GaussianBlur(processed, processed, cv::Size(0, 0), blur_);
    }

    // 用于 AprilTag 检测的“灰度”图像已在 processFrame 中处理。
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

    // 显示处理结果（可选，如需可视化请取消注释）
    cv::imshow("AprilTag Detection", display);
    cv::waitKey(1); // 等待1ms，允许窗口响应事件

    return result;
}

// 处理循环函数实现
void AprilTagTracker::processingLoop()
{
    const int frame_rate = 30;
    cv::Mat frame;

    while (running_)
    {
        // 从仿真环境获取帧
        frame = get_latest_frame();

        if (frame.empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frame_rate));
            continue;
        }

        // 处理单帧图像函数
        AprilTagData result = processFrame(frame);

        {
            std::lock_guard<std::mutex> lock(data_mutex_); // 使用互斥锁保护对最新数据的访问 // 确保线程安全地更新最新检测结果
            latest_data_ = result;                         // 更新最新检测结果
        }

        // 控制帧率
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frame_rate));
    }
}

// 获取最新检测结果实现
AprilTagData AprilTagTracker::getData() const
{
    std::lock_guard<std::mutex> lock(data_mutex_); // 加锁保护数据
    return latest_data_;
}