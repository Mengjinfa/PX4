#ifndef LANDING_STATE_MACHINE_HPP
#define LANDING_STATE_MACHINE_HPP

#include <chrono>
#include <string>

#include "apriltag_tracker.hpp"
#include "flight_procedure.hpp"
#include "mavsdk_members.hpp"
#include "pid.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>

// 定义状态枚举
enum class LandingState
{
    IDLE,
    WAITING,
    ADJUST_POSITION,
    LANDING,
    CIRCLE
};

class LandingStateMachine
{
public:
    LandingStateMachine();

    int StartStateMachine();
    void updateState(Mavsdk_members &mavsdk);

    void setRelevantData(AprilTagData &landmark,
                         PIDOutput &pid_output_,
                         mavsdk::Telemetry::PositionNed &current_position,
                         float &current_yaw_deg,
                         float &current_altitude);

    std::string landingStateToString(const LandingState state);
    LandingState getCurrentStateMachine() const;

private:
    std::chrono::time_point<std::chrono::system_clock> getCurrentTime();
    void waitingState(Mavsdk_members &mavsdk);
    void adjustPositionState(Mavsdk_members &mavsdk);
    void circleState(Mavsdk_members &mavsdk);
    void landingState(Mavsdk_members &mavsdk);

private:
    AprilTagData m_landmark; // 地标数据
    PIDOutput m_pid_out;     // PID输出

    mavsdk::Telemetry::PositionNed m_current_position; // 当前无人机位置(NED坐标系)
    float m_current_altitude = 0.0;                    // 当前相对起飞点的高度(m)
    float m_current_yaw_deg = 0.0;                     // 当前偏航角度(度)

    mavsdk::Telemetry::PositionNed circle_position; // 当前无人机位置(NED坐标系)
    float circle_yaw_deg = 0.0;                     // 当前偏航角度(度)

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

typedef NormalSingleton<LandingStateMachine> landing_state_machine;

#endif
