#include "drone_controller.hpp"
#include "logger.hpp"
#include <Eigen/Dense> // 用于数学计算
#include <cmath>       // 用于数学函数

using namespace mavsdk;
using Clock = std::chrono::steady_clock;
using TimePoint = std::chrono::time_point<Clock>;

// 低通滤波器函数
double lowPassFilter(double current_value, double new_measurement, double alpha)
{
    return alpha * new_measurement + (1 - alpha) * current_value;
}

/**
 * @brief 控制无人机跟踪目标
 *
 * 根据目标在图像中的偏移量计算控制指令，使无人机对准并接近目标
 * @param dx 目标在图像水平方向的相对偏移量（范围：-1.0到1.0）
 * @param dy 目标在图像垂直方向的相对偏移量（范围：-1.0到1.0）
 * @param offboard 无人机板外控制插件引用
 * @param target_position 目标位置
 * @param current_relative_altitude_m 当前相对高度（米）
 */
void DroneController::trackTarget(
    double err_x, double err_y,
    Offboard &offboard,
    const Telemetry::PositionNed &target_position,
    float current_relative_altitude_m)
{
    // 获取当前时间点
    auto current_time = std::chrono::steady_clock::now();

    // 如果是第一次调用，初始化 last_time 并返回
    if (last_time.time_since_epoch().count() == 0)
    {
        last_time = current_time;
        return;
    }

    Offboard::VelocityBodyYawspeed velocity{};                                     // 设置 Offboard 模式下的速度指令
    double dt = std::chrono::duration<double>(current_time - last_time).count();   // 计算自上次调用以来的时间间隔（秒）
    double target_altitude = 0.0;                                                  // 目标高度（假设目标高度为 0）
    double current_altitude_error = current_relative_altitude_m - target_altitude; // 当前高度误差
    last_time = current_time;                                                      // 更新上次调用时间

    // 使用低通滤波器对 dx 和 dy 进行滤波
    filtered_dx = lowPassFilter(filtered_dx, err_x, alpha);
    filtered_dy = lowPassFilter(filtered_dy, err_y, alpha);

    // 根据当前高度计算高度级别
    int height_level = static_cast<int>(current_relative_altitude_m / 0.5);
    double descent_speed = 0.0;               // 下降速度
    double adjusted_position_tolerance = 0.0; // 调整后的定位容忍度

    // 根据高度级别调整下降速度和定位容忍度
    switch (height_level)
    {
        case 0:
            adjusted_position_tolerance = 0.2;
            descent_speed = 0.1;
            break;
        case 1:
            adjusted_position_tolerance = 0.2;
            descent_speed = 0.2;
            break;
        case 2:
            adjusted_position_tolerance = 0.2;
            descent_speed = 0.4;
            break;
        case 3:
            adjusted_position_tolerance = 0.2;
            descent_speed = 0.4;
            break;
        case 4:
            adjusted_position_tolerance = 0.3;
            descent_speed = 0.5;
            break;
        case 5:
            adjusted_position_tolerance = 0.35;
            descent_speed = 0.5;
            break;
        default:
            adjusted_position_tolerance = 0.4;
            descent_speed = 0.6;
            break;
    }

    // 将下降速度减半
    descent_speed *= 0.5;

    // 使用 PID 控制器计算前后、左右和上下方向的速度
    double forward_vel = pid_forward.compute(-filtered_dy, dt); // 前后速度
    double right_vel = pid_right.compute(filtered_dx, dt);      // 左右速度

    // 判断是否处于中心位置
    if (std::abs(filtered_dx) < adjusted_position_tolerance && std::abs(filtered_dy) < adjusted_position_tolerance)
    {
        velocity.forward_m_s = forward_vel; // 前后速度
        velocity.right_m_s = right_vel;     // 左右速度
        velocity.down_m_s = descent_speed;  // 下降速度
        velocity.yawspeed_deg_s = 0.0f;     // 保持航向不变

        // 发送速度指令
        offboard.set_velocity_body(velocity);
    }
    else
    {
        velocity.forward_m_s = forward_vel; // 前后速度
        velocity.right_m_s = right_vel;     // 左右速度
        velocity.down_m_s = -0.001;         // 下降速度
        velocity.yawspeed_deg_s = 0.0f;     // 保持航向不变

        // 发送速度指令
        offboard.set_velocity_body(velocity);
    }
}

/**
 * @brief 控制无人机执行搜索模式
 *
 * 当未检测到目标时，控制无人机按圆轨迹搜索
 * @param elapsed_time 未检测到目标的时间（秒）
 * @param current 当前无人机在NED坐标系中的位置
 * @param offboard 无人机离线控制插件引用
 * @param current_relative_altitude_m 当前相对高度（米）
 */
void DroneController::searchPattern(
    double elapsed_time,
    const Telemetry::PositionNed &current,
    Offboard &offboard,
    float current_relative_altitude_m)
{
    // 如果处于过渡时间范围内
    if (elapsed_time <= transition_time)
    {
        double ratio = elapsed_time / transition_time;         // 计算过渡比例
        double target_angle = ANGULAR_VELOCITY * elapsed_time; // 计算目标角度

        // 计算目标圆周位置
        Eigen::Vector2d target_pos(
            current.north_m + RADIUS * cos(target_angle),
            current.east_m + RADIUS * sin(target_angle));

        // 线性插值当前位置到目标位置
        Eigen::Vector2d current_pos(current.north_m, current.east_m);
        Eigen::Vector2d interpolated = current_pos + ratio * (target_pos - current_pos);

        // 设置 Offboard 模式下的位置指令
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = interpolated.x();
        setpoint.east_m = interpolated.y();
        setpoint.down_m = -current_relative_altitude_m; // 保持当前高度
        setpoint.yaw_deg = 0.0f;                        // 保持航向不变

        // 发送位置指令
        offboard.set_position_ned(setpoint);
    }
    else
    {
        // 如果过渡时间已过，直接计算目标圆周位置
        double target_angle = ANGULAR_VELOCITY * elapsed_time;
        double target_x = current.north_m + RADIUS * cos(target_angle);
        double target_y = current.east_m + RADIUS * sin(target_angle);

        // 设置 Offboard 模式下的位置指令
        Offboard::PositionNedYaw setpoint{};
        setpoint.north_m = target_x;
        setpoint.east_m = target_y;
        setpoint.down_m = -current_relative_altitude_m;
        setpoint.yaw_deg = 0.0f;

        // 发送位置指令
        offboard.set_position_ned(setpoint);
    }
}