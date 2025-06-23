#include "state_machine.hpp"
#include "flight_procedure.hpp"
#include "telemetry_monitor.hpp"

#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

using namespace mavsdk;

StateMachineFlag start_machine_flag;

// 状态机类构造函数，初始化状态机的基本参数和状态
StateMachine::StateMachine()
{
    // 状态机状态初始化
    state_ = LandingState::IDLE;         // 初始状态设为空闲状态
    last_state_ = LandingState::UNKNOWN; // 上一状态初始为未知状态
    start_landing_flag_ = false;         // 降落启动标志初始为false
    landmark_loss_flag_ = false;         // 地标丢失标志初始为false
    start_machine_flag.flag = false;     // 全局状态机标志初始为false
}

// 启动状态机函数
void StateMachine::StartStateMachine(mavsdk::Telemetry::PositionNed current_position, double current_altitude)
{
    m_current_altitude = current_altitude; // 记录当前高度

    // 仅在首次检测到启动标志时记录位置并初始化状态
    if (start_machine_flag.flag && (start_landing_flag_ == false))
    {
        m_current_position = current_position; // 记录当前位置为降落起始点

        state_ = LandingState::WAITING; // 设置为等待状态
        start_landing_flag_ = true;     // 标记已启动降落流程
        std::cout << "降落状态机已启动，初始位置已记录" << std::endl;
    }
}

/**
 * @brief 状态机主更新函数
 * @param current_altitude 当前高度
 * 根据当前状态、高度、地标检测状态等更新状态机状态，
 * 是状态机的核心控制逻辑，根据不同状态调用对应的状态处理函数
 */
void StateMachine::updateState(Mavsdk_members &mavsdk)
{
    // 检测状态是否发生变化
    if (state_ != last_state_)
    {
        last_state_ = state_; // 更新上一状态

        // 若切换到等待状态，记录进入等待状态的时间
        if (state_ == LandingState::WAITING)
        {
            waiting_state_time_ = getCurrentTime();
        }
    }

    // 如果降落标志已启动，根据当前状态执行对应的状态处理
    if (start_landing_flag_)
    {
        switch (state_)
        {
            case LandingState::WAITING:
                waitingState(mavsdk); // 处理等待状态逻辑
                break;
            case LandingState::ADJUST_POSITION:
                adjustPositionState(mavsdk); // 处理位置调整状态逻辑
                break;
            case LandingState::CIRCLE:
                circleState(mavsdk); // 处理绕圈搜索状态逻辑
                break;
            case LandingState::LANDING:
                landingState(mavsdk); // 处理降落状态逻辑
                break;
            default: // 异常状态处理
                break;
        }
    }
}

/**
 * @brief 等待状态处理函数
 * @param current_altitude 当前高度
 * 等待状态主要用于检测地标稳定性：
 * 1. 持续5秒内检测地标稳定性
 * 2. 若稳定检测到地标则进入位置调整状态
 * 3. 若未稳定检测到地标则进入绕圈搜索状态
 */
void StateMachine::waitingState(Mavsdk_members &mavsdk)
{
    // 保持当前位置
    offboard_flight_position(mavsdk, m_current_position.north_m, m_current_position.east_m, m_current_position.down_m, 90.0);

    // 检查是否已等待5秒
    if (std::chrono::duration_cast<std::chrono::duration<double>>(getCurrentTime() - waiting_state_time_).count() >= 5.0)
    {
        // 根据地标检测次数决定下一状态
        if (landmark_detection_count_ > 30)
        {
            state_ = LandingState::ADJUST_POSITION; // 检测次数足够，切换到位置调整状态
        }
        else
        {
            state_ = LandingState::CIRCLE; // 切换到绕圈搜索状态
        }
        landmark_detection_count_ = 0; // 重置检测计数
    }
    else
    {
        // 若检测到地标则增加检测计数
        if (landmark_.iffind)
        {
            landmark_detection_count_++; // 地标可见，计数增加
        }
    }
}

/**
 * @brief 位置调整状态处理函数
 * @param current_altitude 当前高度
 * 位置调整状态主要用于对准地标：
 * 1. 根据高度分层设置不同的控制参数
 * 2. 若误差在容差内则开始下降，否则水平移动对准地标
 * 3. 地标丢失时切换到绕圈搜索状态
 */
void StateMachine::adjustPositionState(Mavsdk_members &mavsdk)
{
    // 当高度高于0.5米时执行位置调整
    if (m_current_altitude > 0.5)
    {
        int height_level = static_cast<int>(m_current_altitude / 0.5); // 按0.5米分层计算高度层级
        int position_tolerance = 0;                                    // 位置容忍度
        double descent_speed = 0.0;                                    // 下降速度

        // 根据高度层级设置不同的位置容忍度和下降速度
        switch (height_level)
        {
            case 0:
                position_tolerance = 30; // 0-0.5米高度范围
                descent_speed = 0.3;
                break;
            case 1:
                position_tolerance = 40; // 0.5-1米高度范围
                descent_speed = 0.3;
                break;
            case 2:
                position_tolerance = 40; // 1-1.5米高度范围
                descent_speed = 0.4;
                break;
            case 3:
                position_tolerance = 50; // 1.5-2米高度范围
                descent_speed = 0.4;
                break;
            case 4:
                position_tolerance = 60; // 2-2.5米高度范围
                descent_speed = 0.5;
                break;
            case 5:
                position_tolerance = 70; // 2.5-3米高度范围
                descent_speed = 0.5;
                break;
            default:
                position_tolerance = 80; // 3米以上高度范围
                descent_speed = 0.6;
                break;
        }

        // descent_speed = descent_speed * 0.5; // 调整下降速度

        // 处理地标可见的情况
        if (landmark_.iffind)
        {
            // 根据误差与容忍度的关系决定是下降还是水平移动
            if (std::abs(landmark_.err_x) < position_tolerance && std::abs(landmark_.err_y) < position_tolerance)
            {
                // 误差在容忍度内，开始下降
                offboard_flight_body_velocity(mavsdk, PID_out_.x, PID_out_.y, descent_speed, 0.0);
            }
            else
            {
                // 误差超出容忍度，水平移动调整位置
                offboard_flight_body_velocity(mavsdk, PID_out_.x, PID_out_.y, 0.01, 0.0);
            }

            // 检测到地标，重置丢失标志
            landmark_loss_flag_ = false;
        }
        else // 处理地标丢失的情况
        {
            if (!landmark_loss_flag_)
            {
                // 首次检测到地标丢失，记录时间
                landmark_loss_flag_ = true;
                landmark_loss_start_time_ = getCurrentTime();
            }
            else
            {
                // 地标丢失超过3秒，切换到绕圈搜索状态
                if (std::chrono::duration_cast<std::chrono::duration<double>>(getCurrentTime() - landmark_loss_start_time_).count() >= 3.0)
                {
                    state_ = LandingState::CIRCLE;
                    landmark_loss_flag_ = false;
                }
            }
        }
    }
    else
    {
        state_ = LandingState::LANDING; // 高度过低，进入降落状态
    }
}

/**
 * @brief 绕圈搜索状态处理函数
 * @param current_altitude 当前高度
 * 绕圈搜索状态主要用于地标丢失时的搜索：
 * 1. 在安全高度保持绕圈搜索地标
 * 2. 检测到地标后切换回位置调整状态
 */
void StateMachine::circleState(Mavsdk_members &mavsdk)
{
    // 当高度大于1.0米时执行绕圈搜索
    if (m_current_altitude > 1.0)
    {
        static bool first_entry = true;
        static double start_angle = 0.0;
        static std::chrono::time_point<std::chrono::system_clock> circle_start_time;

        // 首次进入绕圈搜索状态时的初始化
        if (first_entry)
        {
            circle_start_time = getCurrentTime();
            first_entry = false;
        }

        // 计算绕圈时间
        double elapsed_time = std::chrono::duration_cast<std::chrono::duration<double>>(getCurrentTime() - circle_start_time).count();
        const double transition_time = 5.0; // 加速阶段持续时间

        // 如果处于过渡时间范围内
        if (elapsed_time <= transition_time)
        {
            // 设置 Offboard 模式下的位置指令
            Offboard::PositionNedYaw setpoint_1{};

            double ratio = elapsed_time / transition_time;
            double target_angle = ANGULAR_VELOCITY * elapsed_time;
            double target_x = m_current_position.north_m + RADIUS * cos(target_angle);
            double target_y = m_current_position.east_m + RADIUS * sin(target_angle);

            // 平滑过渡到目标圆周
            setpoint_1.north_m = m_current_position.north_m + ratio * (target_x - m_current_position.north_m);
            setpoint_1.east_m = m_current_position.east_m + ratio * (target_y - m_current_position.east_m);
            setpoint_1.down_m = m_current_position.down_m;

            offboard_flight_position(mavsdk, setpoint_1.north_m, setpoint_1.east_m, setpoint_1.down_m, 90.0); // 发送位置指令
        }
        else
        {
            // 设置 Offboard 模式下的位置指令
            Offboard::PositionNedYaw setpoint_2{};

            double angle = start_angle + ANGULAR_VELOCITY * (elapsed_time - transition_time);

            // 平滑过渡到目标圆周
            setpoint_2.north_m = m_current_position.north_m + RADIUS * cos(angle);
            setpoint_2.east_m = m_current_position.east_m + RADIUS * sin(angle);
            setpoint_2.down_m = m_current_position.down_m;

            offboard_flight_position(mavsdk, setpoint_2.north_m, setpoint_2.east_m, setpoint_2.down_m, 90.0); // 发送位置指令
        }

        // 检测到地标时切换回位置调整状态
        if (landmark_.iffind)
        {
            state_ = LandingState::ADJUST_POSITION;
            first_entry = true; // 重置首次进入标志
        }
    }
    else
    {
        state_ = LandingState::LANDING; // 高度过低，进入降落状态
    }
}

/**
 * @brief 降落状态处理函数
 * @param current_altitude 当前高度
 * 降落状态主要用于执行着陆流程：
 * 1. 高度>0.5米时缓慢下降并保持对准地标
 * 2. 高度≤0.5米时切换至自动着陆模式
 */
void StateMachine::landingState(Mavsdk_members &mavsdk)
{
    Action &action = mavsdk.action;

    static bool timer_started_ = false;
    static std::chrono::time_point<std::chrono::system_clock> landing_start_time;

    // 首次进入降落状态时记录时间
    if (!timer_started_)
    {
        landing_start_time = getCurrentTime();
        timer_started_ = true;
    }

    // 高度大于0.5米且降落时间小于5秒时，继续调整降落
    if (m_current_altitude > 0.5 && std::chrono::duration_cast<std::chrono::duration<double>>(getCurrentTime() - landing_start_time).count() < 5.0)
    {
        // 地标可见时，带位置调整下降
        if (landmark_.iffind)
        {
            offboard_flight_body_velocity(mavsdk, PID_out_.x, PID_out_.y, 0.2, 0.0); // 下降速度0.2m/s
        }
        else // 地标丢失时，垂直下降
        {
            offboard_flight_body_velocity(mavsdk, 0.0, 0.0, 0.2, 0.0);
        }
    }
    else // 切换到自动降落模式
    {
        action.land();
        std::cout << "切换到 自动降落模式" << std::endl; // 输出模式切换信息
        timer_started_ = false;                          // 重置计时器标志
        start_landing_flag_ = false;                     // 关闭降落标志
    }
}

/**
 * @brief 获取地标信息
 * @param landmark 地标数据结构
 * 更新当前检测到的地标数据
 */
void StateMachine::getLandmark(const AprilTagData &landmark)
{
    landmark_ = landmark; // 更新地标数据
}

/**
 * @brief 更新当前PID的计算结果
 * @param pid_output_ PID输出结构体
 * @note const表示在函数内部不会修改这个参数的值，&则表示使用引用传递，避免了对象的拷贝，提高了效率。
 */
void StateMachine::getPIDOut(const PIDOutput &pid_output_)
{
    PID_out_ = pid_output_; // 更新PID输出
}

/**
 * @brief 将降落状态枚举转换为字符串
 * @param state 降落状态枚举值
 * @return 对应的状态字符串
 * @note 函数后面的const说明在调用该函数时，不会改变对象的状态。
 */
std::string StateMachine::landingStateToString(LandingState state) const
{
    // 根据状态枚举值返回对应的字符串表示
    switch (state)
    {
        case LandingState::IDLE:
            return "IDLE";
        case LandingState::WAITING:
            return "WAITING";
        case LandingState::ADJUST_POSITION:
            return "ADJUST_POSITION";
        case LandingState::LANDING:
            return "LANDING";
        case LandingState::CIRCLE:
            return "CIRCLE";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief 获取当前状态
 * @return 当前状态枚举值
 */
LandingState StateMachine::getCurrentStateMachine() const
{
    return state_; // 返回当前状态
}

// 获取当前时间点
std::chrono::time_point<std::chrono::system_clock> StateMachine::getCurrentTime()
{
    return std::chrono::system_clock::now();
}