#ifndef DETECTIONSTATEMACHINE_H
#define DETECTIONSTATEMACHINE_H

#include <atomic>
#include <chrono>

// 假设 DroneState 是一个枚举类型
enum class DroneState
{
    TRACKING,
    SEARCHING,
    LANDING,
    NOT_DETECTED
};

class DetectionStateMachine
{
public:
    DetectionStateMachine();

    DroneState update(bool detected);
    bool isLandingComplete() const;

    void setCurrentRelativeAltitude(float altitude);
    float getNotDetectedAltitude();

private:
    DroneState state = DroneState::TRACKING;
    std::chrono::steady_clock::time_point no_detection_start;
    std::atomic<bool> landing_complete; // 降落完成标志

    int detection_count = 0;                                                                            // 检测到目标的次数
    int no_detection_count = 0;                                                                         // 连续未检测到目标的次数
    std::chrono::time_point<std::chrono::steady_clock> search_start = std::chrono::steady_clock::now(); // 记录 SEARCHING 开始时间
    const int timeout = 5;                                                                              // 超时时间（秒）
    bool has_tracked_once = false;
    float saved_altitude = 0.0f; // 保存高度值
    float in_altitude = 0.0f;    // 外部传入高度值
};

#endif // DETECTIONSTATEMACHINE_H