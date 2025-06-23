#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "apriltag_tracker.hpp"
#include "mavsdk_members.hpp"
#include "pid.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <cmath>
#include <string>

// 定义状态枚举
enum class LandingState
{
    IDLE,
    WAITING,
    ADJUST_POSITION,
    LANDING,
    CIRCLE,
    UNKNOWN
};

// PID输出结构
struct StateMachineFlag
{
    bool flag; // 控制标志，指示是否需要控制
};
extern StateMachineFlag start_machine_flag; // 全局状态机标志

class StateMachine
{
public:
    StateMachine();

    void StartStateMachine(mavsdk::Telemetry::PositionNed current_position, double current_altitude); // 启动状态机
    void updateState(Mavsdk_members &mavsdk);                                                         // 更新状态机状态

    void getLandmark(const AprilTagData &landmark);             // 获取地标数据
    void getPIDOut(const PIDOutput &pid_output_);               // 获取PID计算结果
    std::string landingStateToString(LandingState state) const; // 获取降落状态字符串表示
    LandingState getCurrentStateMachine() const;                // 获取当前状态

private:
    std::chrono::time_point<std::chrono::system_clock> getCurrentTime(); // 获取当前时间点
    void waitingState(Mavsdk_members &mavsdk);                           // 等待状态：检测地标稳定性
    void adjustPositionState(Mavsdk_members &mavsdk);                    // 位置调整状态：对准地标
    void circleState(Mavsdk_members &mavsdk);                            // 绕圈搜索状态：地标丢失时搜索
    void landingState(Mavsdk_members &mavsdk);                           // 降落状态：执行着陆流程

private:
    AprilTagData landmark_; // 地标数据
    PIDOutput PID_out_;     // PID输出

    mavsdk::Telemetry::PositionNed m_current_position; // 当前无人机位置(NED坐标系)
    double m_current_altitude;                         // 当前相对起飞点的高度(m

    LandingState state_;      // 当前状态
    LandingState last_state_; // 上一状态
    bool start_landing_flag_; // 降落启动标志
    bool landmark_loss_flag_; // 地标丢失检测标志

    std::chrono::time_point<std::chrono::system_clock> waiting_state_time_;
    std::chrono::time_point<std::chrono::system_clock> landmark_loss_start_time_;

    double ANGULAR_VELOCITY = 0.5; // 绕圈搜索时的角速度，单位：rad/s
    double RADIUS = 0.5;           // 绕圈搜索的半径，单位：米

    int landmark_detection_count_ = 0; // 地标检测计数
};

#endif
