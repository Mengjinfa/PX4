#ifndef APRILTAG_TRACKER_HPP
#define APRILTAG_TRACKER_HPP

#include "apriltag/apriltag.h"
#include "apriltag/tag25h9.h"
#include "singleton.hpp"

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
    float x, y;                    // 标签中心坐标
    int width, height;             // 图像宽高
    double err_x, err_y;           // 与图像中心的偏差
    double norm_err_x, norm_err_y; // 归一化偏差
    float size;                    // 标签大小
};

// 获取最新帧函数（仿真模式专用）
cv::Mat get_latest_frame();

class AprilTagTracker
{
public:
    AprilTagTracker();  // 初始化AprilTag跟踪器，设置AprilTag检测器
    ~AprilTagTracker(); // 清理资源，停止处理线程，销毁AprilTag检测器

    void GazeboStart(int argc, char *argv[]); // 在单独线程中启动AprilTag处理循环
    AprilTagData process();                   // 持续从仿真环境获取图像并检测AprilTag

    AprilTagData detect(cv::Mat &frame, bool drawOverlay = true);

private:
    cv::Mat preprocessImage(const cv::Mat &frame) const;            // 对输入图像进行增强和降噪处理，提高AprilTag检测成功率
    double calculateTagArea(const apriltag_detection_t *det) const; // 计算标签面积（凸四边形面积公式）
    AprilTagData processFrame(const cv::Mat &frame) const;          // 检测图像中的AprilTag，返回检测结果并绘制可视化结果

private:
    clock_t _start;                 // 用于计时
    std::vector<float> _areas;      // 保存检测到的标签面积
    std::atomic<bool> running_;     // 运行状态标志
    mutable std::mutex data_mutex_; // 数据互斥锁
    AprilTagData _last_results;     // 最新检测数据

    // AprilTag检测器相关
    apriltag_detector_t *td; // AprilTag检测器
    apriltag_family_t *tf;   // 标签家族
};

typedef NormalSingleton<AprilTagTracker> tag_tracker;

#endif // APRILTAG_TRACKER_H