#include "drone_controller.hpp"
#include <cmath>

#include "logger.hpp" // 使用全局日志模块

using namespace mavsdk;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// 低通滤波器
double lowPassFilter(double current_value, double new_measurement, double alpha)
{
    return alpha * new_measurement + (1 - alpha) * current_value;
}

void DroneController::trackTarget(
    double dx, double dy,
    Offboard &offboard,
    const Telemetry::PositionNed &target_position,
    float current_relative_altitude_m)
{

    auto current_time = std::chrono::steady_clock::now();
    if (last_time.time_since_epoch().count() == 0)
    {
        last_time = current_time;
        return;
    }
    double dt = std::chrono::duration<double>(current_time - last_time).count();
    last_time = current_time;

    double target_altitude = 0.0;
    double current_altitude_error = current_relative_altitude_m - target_altitude;

    logging::get_logger()->info("当前高度: {}", current_relative_altitude_m);
    logging::get_logger()->info("原始偏移 dx: {}, dy: {}", dx, dy);

    filtered_dx = lowPassFilter(filtered_dx, dx, alpha);
    filtered_dy = lowPassFilter(filtered_dy, dy, alpha);

    logging::get_logger()->info("滤波后偏移 dx: {}, dy: {}", filtered_dx, filtered_dy);

    int height_level = static_cast<int>(current_relative_altitude_m / 0.5);
    double descent_speed = 0.0;
    double adjusted_position_tolerance = 0.0;

    switch (height_level)
    {
        case 0:
            adjusted_position_tolerance = 8;
            descent_speed = 0.1;
            break;
        case 1:
            adjusted_position_tolerance = 15;
            descent_speed = 0.2;
            break;
        case 2:
            adjusted_position_tolerance = 20;
            descent_speed = 0.4;
            break;
        case 3:
            adjusted_position_tolerance = 30;
            descent_speed = 0.4;
            break;
        case 4:
            adjusted_position_tolerance = 40;
            descent_speed = 0.5;
            break;
        case 5:
            adjusted_position_tolerance = 50;
            descent_speed = 0.5;
            break;
        default:
            adjusted_position_tolerance = 50;
            descent_speed = 0.6;
            break;
    }

    descent_speed *= 0.5;

    double forward_vel = pid_forward.compute(-filtered_dy, dt);
    double right_vel = pid_right.compute(filtered_dx, dt);
    double down_vel = pid_down.compute(current_altitude_error, dt);
    static double filtered_down_vel = 0.0;
    double alpha_down = 0.2;
    filtered_down_vel = lowPassFilter(filtered_down_vel, down_vel, alpha_down);
    filtered_down_vel = std::clamp(filtered_down_vel, 0.0, descent_speed);

    double position_tolerance = adjusted_position_tolerance / 100.0 * 0.50;
    bool is_centered = (std::abs(filtered_dx) < position_tolerance) && (std::abs(filtered_dy) < position_tolerance);

    Offboard::VelocityBodyYawspeed velocity{};
    velocity.forward_m_s = forward_vel;
    velocity.right_m_s = right_vel;
    velocity.down_m_s = is_centered ? filtered_down_vel : 0.0;
    velocity.yawspeed_deg_s = 0.0f;

    offboard.set_velocity_body(velocity);

    logging::get_logger()->info("向前速度: {}", forward_vel);
    logging::get_logger()->info("向右速度: {}", right_vel);
    logging::get_logger()->info("向下速度: {}", velocity.down_m_s);
    logging::get_logger()->info("忍受区间: {}", position_tolerance);
}

void DroneController::searchPattern(
    double elapsed,
    const Telemetry::PositionNed &current,
    Offboard &offboard,
    float current_relative_altitude_m)
{

    logging::get_logger()->info("传入高度值: {}", current_relative_altitude_m);
    logging::get_logger()->info("传入时间: {}", elapsed);

    if (elapsed <= transition_time)
    {
        double ratio = elapsed / transition_time;
        double target_angle = ANGULAR_VELOCITY * elapsed;
        double target_x = current.north_m + RADIUS * cos(target_angle);
        double target_y = current.east_m + RADIUS * sin(target_angle);

        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = target_x + ratio * (target_x - current.north_m);
        setpoint.east_m = target_y + ratio * (target_y - current.east_m);
        setpoint.down_m = -current_relative_altitude_m;
        setpoint.yaw_deg = 0.0f;
        offboard.set_position_ned(setpoint);
    }
    else
    {
        double angle = fmod(start_angle + ANGULAR_VELOCITY * (elapsed - transition_time), 2 * M_PI);
        double target_x = current.north_m + RADIUS * cos(angle);
        double target_y = current.east_m + RADIUS * sin(angle);
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = target_x;
        setpoint.east_m = target_y;
        setpoint.down_m = -current_relative_altitude_m;
        setpoint.yaw_deg = 0.0f;
        offboard.set_position_ned(setpoint);
    }
}