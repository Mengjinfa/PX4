#include "detection_state_machine.hpp"
#include <chrono>

// DetectionStateMachine 类的默认构造函数
DetectionStateMachine::DetectionStateMachine() = default;

// 设置当前相对高度（来自无人机遥测数据）
void DetectionStateMachine::setCurrentRelativeAltitude(float altitude)
{
    in_altitude = altitude; // 将传入的高度值赋给成员变量 in_altitude
}

// 获取未检测到目标时的保存高度（用于搜索模式）
float DetectionStateMachine::getNotDetectedAltitude()
{
    return saved_altitude; // 返回保存的高度值
}

// 检查降落是否完成
bool DetectionStateMachine::isLandingComplete() const
{
    return landing_complete.load(); // 返回 landing_complete 的值
}

// 更新状态机的状态，根据是否检测到目标来决定下一步操作
DroneState DetectionStateMachine::update(bool detected)
{
    auto now = std::chrono::steady_clock::now(); // 获取当前时间点

    // 根据当前状态进行分支处理
    switch (state)
    {
        case DroneState::SEARCHING: // 当前状态为搜索模式
        {
            if (detected) // 如果检测到目标
            {
                detection_count++;                                  // 增加检测计数
                if (detection_count >= (has_tracked_once ? 1 : 30)) // 如果检测计数达到阈值
                {
                    state = DroneState::TRACKING; // 切换到跟踪模式
                    detection_count = 0;          // 重置检测计数
                    no_detection_count = 0;       // 重置未检测计数
                    has_tracked_once = true;      // 标记已经跟踪过一次
                }
            }
            else // 如果未检测到目标
            {
                if (std::chrono::duration_cast<std::chrono::seconds>(now - search_start).count() >= 5) // 如果搜索时间超过 5 秒
                {
                    state = DroneState::NOT_DETECTED; // 切换到未检测模式
                    no_detection_count = 0;           // 重置未检测计数
                    detection_count = 0;              // 重置检测计数
                    saved_altitude = in_altitude;     // 保存当前高度
                }
            }
        }
        break;

        case DroneState::TRACKING: // 跟踪模式
        {
            if (detected) // 如果检测到目标
            {
                no_detection_count = 0; // 重置未检测计数
            }
            else // 如果未检测到目标
            {
                no_detection_count++;         // 增加未检测计数
                if (no_detection_count >= 20) // 如果未检测计数达到阈值
                {
                    state = DroneState::SEARCHING;                   // 切换到搜索模式
                    search_start = std::chrono::steady_clock::now(); // 重置搜索开始时间
                    no_detection_count = 0;                          // 重置未检测计数
                    detection_count = 0;                             // 重置检测计数
                    saved_altitude = in_altitude;                    // 保存当前高度
                }
            }
        }
        break;

        case DroneState::NOT_DETECTED: // 未检测模式
        {
            if (detected) // 如果检测到目标
            {
                state = DroneState::SEARCHING;                   // 切换到搜索模式
                search_start = std::chrono::steady_clock::now(); // 重置搜索开始时间
                saved_altitude = in_altitude;                    // 保存当前高度
            }
        }
        break;

        case DroneState::LANDING: // 降落模式
        {
        }
        break;

        default: // 处理其他未明确列出的状态
            break;
    }

    return state; // 返回当前状态
}
