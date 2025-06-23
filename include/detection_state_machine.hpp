#ifndef DETECTIONSTATEMACHINE_H
#define DETECTIONSTATEMACHINE_H

#include <atomic>
#include <chrono>

// 定义无人机的状态枚举
enum class DroneState
{
    TRACKING,    // 跟踪状态
    SEARCHING,   // 搜索状态
    LANDING,     // 降落状态
    NOT_DETECTED // 未检测到目标状态
};

// DetectionStateMachine 类定义
class DetectionStateMachine
{
public:
    DetectionStateMachine(); // 默认构造函数

    DroneState update(bool detected); // 更新状态机的状态，根据是否检测到目标来决定下一步操作

    bool isLandingComplete() const;                  // 检查降落是否完成
    void setCurrentRelativeAltitude(float altitude); // 设置当前相对高度（来自无人机遥测数据）
    float getNotDetectedAltitude();                  // 获取未检测到目标时的保存高度（用于搜索模式）

private:
    DroneState state = DroneState::TRACKING;                  // 当前状态，默认为 TRACKING
    std::chrono::steady_clock::time_point no_detection_start; // 未检测到目标的开始时间点
    std::atomic<bool> landing_complete;                       // 降落完成标志，使用原子类型确保线程安全

    // 搜索模式的开始时间点
    std::chrono::time_point<std::chrono::steady_clock> search_start = std::chrono::steady_clock::now();

    int detection_count = 0;       // 检测到目标的计数
    int no_detection_count = 0;    // 未检测到目标的计数
    const int timeout = 5;         // 超时时间（秒）
    bool has_tracked_once = false; // 是否已经跟踪过一次目标
    float saved_altitude = 0.0f;   // 未检测到目标时保存的高度
    float in_altitude = 0.0f;      // 当前相对高度
};

#endif // DETECTIONSTATEMACHINE_H