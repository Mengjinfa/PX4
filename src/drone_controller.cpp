#include "drone_controller.hpp"
#include "logger.hpp"
#include <cmath>

using namespace mavsdk;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// 低通滤波器
double lowPassFilter(double current_value, double new_measurement, double alpha)
{
    return alpha * new_measurement + (1 - alpha) * current_value;
}

// 目标跟踪函数：根据视觉识别的偏移量控制无人机移动
void DroneController::trackTarget(
    double dx, double dy,                          // 目标在图像中的x/y偏移量(像素)
    Offboard &offboard,                            // MAVSDK离板控制插件
    const Telemetry::PositionNed &target_position, // 目标位置(NED坐标系)
    float current_relative_altitude_m)             // 当前相对高度(米)
{
    // 计算时间间隔，用于PID控制器
    auto current_time = std::chrono::steady_clock::now();
    if (last_time.time_since_epoch().count() == 0)
    {
        last_time = current_time;
        return;
    }
    double dt = std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;

    // 目标高度与高度误差计算
    double target_altitude = 0.0;
    double current_altitude_error = current_relative_altitude_m - target_altitude;

    // 记录原始数据（用于调试）
    logging::get_logger()->info("当前高度: {}", current_relative_altitude_m);
    logging::get_logger()->info("原始偏移 dx: {}, dy: {}", dx, dy);

    // 应用低通滤波处理偏移量，减少抖动
    filtered_dx = lowPassFilter(filtered_dx, dx, alpha);
    filtered_dy = lowPassFilter(filtered_dy, dy, alpha);

    logging::get_logger()->info("滤波后偏移 dx: {}, dy: {}", filtered_dx, filtered_dy);

    // 根据当前高度确定控制参数（高度分层处理）
    int height_level = static_cast<int>(current_relative_altitude_m / 0.5);
    double descent_speed = 0.0;               // 下降速度
    double adjusted_position_tolerance = 0.0; // 位置容差(像素)

    // 高度分层策略：高度越低，容差越小，下降速度越慢
    switch (height_level)
    {
        case 0: // 0-0.5m
            adjusted_position_tolerance = 8;
            descent_speed = 0.1;
            break;
        case 1: // 0.5-1.0m
            adjusted_position_tolerance = 15;
            descent_speed = 0.2;
            break;
        case 2: // 1.0-1.5m
            adjusted_position_tolerance = 20;
            descent_speed = 0.4;
            break;
        case 3: // 1.5-2.0m
            adjusted_position_tolerance = 30;
            descent_speed = 0.4;
            break;
        case 4: // 2.0-2.5m
            adjusted_position_tolerance = 40;
            descent_speed = 0.5;
            break;
        case 5: // 2.5-3.0m
            adjusted_position_tolerance = 50;
            descent_speed = 0.5;
            break;
        default: // >3.0m
            adjusted_position_tolerance = 50;
            descent_speed = 0.6;
            break;
    }

    descent_speed *= 0.5;

    // PID控制器计算三个方向的速度
    double forward_vel = pid_forward.compute(-filtered_dy, dt);     // 前后方向(负dy表示向前)
    double right_vel = pid_right.compute(filtered_dx, dt);          // 左右方向
    double down_vel = pid_down.compute(current_altitude_error, dt); // 上下方向

    // 对下降速度进行滤波和平滑限制
    static double filtered_down_vel = 0.0;
    double alpha_down = 0.2;
    filtered_down_vel = lowPassFilter(filtered_down_vel, down_vel, alpha_down);
    filtered_down_vel = std::clamp(filtered_down_vel, 0.0, descent_speed);

    // 转换位置容差
    double position_tolerance = adjusted_position_tolerance / 100.0 * 0.50;

    // 判断目标是否在容差范围内
    bool is_centered = (std::abs(filtered_dx) < position_tolerance) && (std::abs(filtered_dy) < position_tolerance);

    // 构建速度控制指令（机体坐标系）
    Offboard::VelocityBodyYawspeed velocity{};
    velocity.forward_m_s = forward_vel;                        // 前向速度
    velocity.right_m_s = right_vel;                            // 右向速度
    velocity.down_m_s = is_centered ? filtered_down_vel : 0.0; // 仅当目标居中时下降
    velocity.yawspeed_deg_s = 0.0f;                            // 偏航角速度
    offboard.set_velocity_body(velocity);                      // 发送控制指令到无人机

    // 记录控制参数（用于调试）
    logging::get_logger()->info("向前速度: {}", forward_vel);
    logging::get_logger()->info("向右速度: {}", right_vel);
    logging::get_logger()->info("向下速度: {}", velocity.down_m_s);
    logging::get_logger()->info("忍受区间: {}", position_tolerance);
}

// 搜索模式函数：控制无人机以圆形轨迹搜索目标
void DroneController::searchPattern(
    double elapsed,                        // 自搜索开始的时间(秒)
    const Telemetry::PositionNed &current, // 当前位置(NED坐标系)
    Offboard &offboard,                    // MAVSDK离板控制插件
    float current_relative_altitude_m)     // 当前相对高度(米)
{
    // 记录调试信息
    logging::get_logger()->info("传入高度值: {}", current_relative_altitude_m);
    logging::get_logger()->info("传入时间: {}", elapsed);

    // 搜索模式分阶段处理：初始过渡阶段和稳定圆周运动阶段
    if (elapsed <= transition_time)
    {
        // 过渡阶段：从当前位置平滑过渡到圆周轨迹
        double ratio = elapsed / transition_time;                       // 过渡比例
        double target_angle = ANGULAR_VELOCITY * elapsed;               // 当前目标角度
        double target_x = current.north_m + RADIUS * cos(target_angle); // 目标X坐标
        double target_y = current.east_m + RADIUS * sin(target_angle);  // 目标Y坐标

        // 构建位置控制指令（平滑插值）
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = target_x + ratio * (target_x - current.north_m);
        setpoint.east_m = target_y + ratio * (target_y - current.east_m);
        setpoint.down_m = -current_relative_altitude_m; // 负号表示向下（NED坐标系）
        setpoint.yaw_deg = 0.0f;                        // 偏航角

        // 发送位置控制指令
        offboard.set_position_ned(setpoint);
    }
    else
    {
        // 稳定圆周运动阶段：以固定半径和角速度绕圈
        double angle = fmod(start_angle + ANGULAR_VELOCITY * (elapsed - transition_time), 2 * M_PI);
        double target_x = current.north_m + RADIUS * cos(angle); // 计算圆形轨迹上的目标X
        double target_y = current.east_m + RADIUS * sin(angle);  // 计算圆形轨迹上的目标Y

        // 构建位置控制指令
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = target_x;
        setpoint.east_m = target_y;
        setpoint.down_m = -current_relative_altitude_m;
        setpoint.yaw_deg = 0.0f;

        // 发送位置控制指令
        offboard.set_position_ned(setpoint);
    }
}