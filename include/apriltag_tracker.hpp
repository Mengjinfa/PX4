#ifndef APRILTAG_TRACKER_HPP
#define APRILTAG_TRACKER_HPP

#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>

// AprilTag检测结果结构体
struct AprilTagData
{
    bool iffind;                   // 是否检测到标签
    int id;                        // 标签ID
    int x, y;                      // 标签中心坐标
    int width, height;             // 图像宽高
    double err_x, err_y;           // 与图像中心的偏差
    double norm_err_x, norm_err_y; // 归一化偏差
    bool detected;                 // 是否检测到标签
};

// 获取最新帧函数（仿真模式专用）
cv::Mat get_latest_frame();

class AprilTagTracker
{
public:
    AprilTagTracker();  // 初始化AprilTag跟踪器，设置AprilTag检测器
    ~AprilTagTracker(); // 清理资源，停止处理线程，销毁AprilTag检测器

    void start(int argc, char *argv[]); // 在单独线程中启动AprilTag处理循环
    void stop();                        // 停止处理线程

    AprilTagData getData() const; // 获取最新检测结果

    /**
     * @brief 设置参数（可扩展接口）
     */
    void setDecimate(double decimate);
    void setBlur(double blur);
    void setThreads(int threads);
    void setRefineEdges(bool refine);

private:
    void processingLoop();                                          // 持续从仿真环境获取图像并检测AprilTag
    cv::Mat preprocessImage(const cv::Mat &frame) const;            // 对输入图像进行增强和降噪处理，提高AprilTag检测成功率
    double calculateTagArea(const apriltag_detection_t *det) const; // 计算标签面积（凸四边形面积公式）
    AprilTagData processFrame(const cv::Mat &frame) const;          // 检测图像中的AprilTag，返回检测结果并绘制可视化结果

private:
    std::atomic<bool> running_;     // 运行状态标志
    std::thread processing_thread_; // 处理线程
    mutable std::mutex data_mutex_; // 数据互斥锁
    AprilTagData latest_data_;      // 最新检测数据

    // AprilTag检测器相关
    apriltag_detector_t *td_; // AprilTag检测器
    apriltag_family_t *tf_;   // 标签家族

    // 检测参数
    double decimate_;   // 图像降采样因子
    double blur_;       // 高斯模糊系数
    int threads_;       // 线程数
    bool refine_edges_; // 边缘优化
};

#endif // APRILTAG_TRACKER_H