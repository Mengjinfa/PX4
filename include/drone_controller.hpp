#pragma once // 确保头文件只被包含一次
#include "target_tracker.hpp"
#include <chrono>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

struct PIDController
{
    double kp, ki, kd; // PID 控制器的参数：比例、积分、微分
    double integral;   // 积分项
    double last_error; // 上一次的误差

    // 构造函数，初始化 PID 参数
    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), integral(0.0), last_error(0.0) {}

    // 计算控制输出
    double compute(double error, double dt)
    {
        double derivative = 0.0; // 微分项
        if (dt > 0)
        {
            derivative = (error - last_error) / dt; // 计算微分
            integral += error * dt;                 // 累加积分

            // 限制积分项的范围
            integral = std::clamp(integral, -1.0, 1.0);
        }

        last_error = error;                                  // 更新上一次的误差
        return kp * error + ki * integral + kd * derivative; // 返回控制输出
    }

    // 重置 PID 控制器
    void reset()
    {
        integral = 0.0;   // 清零积分项
        last_error = 0.0; // 清零上一次的误差
    }
};

class DroneController
{
private:
    static constexpr double ANGULAR_VELOCITY_DEFAULT = 0.2; // 默认角速度
    static constexpr double TRANSITION_TIME_DEFAULT = 5.0;  // 默认过渡时间

    const float MAX_SPEED;                                    // 最大速度
    const float RADIUS;                                       // 半径
    const double ANGULAR_VELOCITY = ANGULAR_VELOCITY_DEFAULT; // 角速度
    const double transition_time = TRANSITION_TIME_DEFAULT;   // 过渡时间

    // PID 控制器参数
    PIDController pid_forward{0.8, 0.0, 0.01}; // 前后方向
    PIDController pid_right{0.8, 0.0, 0.01};   // 左右方向

    double alpha = 0.2;       // 滤波系数，越小越平滑
    double filtered_dx = 0.0; // 滤波后的值
    double filtered_dy = 0.0;

    // 上次时间戳
    std::chrono::steady_clock::time_point last_time;

public:
    /**
     * @brief 构造函数，初始化最大速度和半径
     * @param MAX_SPEED(max_speed)：初始化常量成员变量 MAX_SPEED，该值在对象生命周期内不可修改
     * @param RADIUS(radius)：初始化常量成员 RADIUS。
     * @param last_time(...)：使用 std::chrono 库记录对象创建时的时间点，用于后续的时间差计算
     */
    explicit DroneController(float max_speed, float radius) : MAX_SPEED(max_speed), RADIUS(radius), last_time(std::chrono::steady_clock::now())
    {
        // 确保 alpha 合法
        alpha = std::clamp(alpha, 0.0, 1.0);
    }

    // 跟踪目标
    void trackTarget(double dx, double dy, mavsdk::Offboard &offboard, const mavsdk::Telemetry::PositionNed &current, float current_relative_altitude_m);

    // 执行搜索模式
    void searchPattern(double elapsed_time, const mavsdk::Telemetry::PositionNed &current, mavsdk::Offboard &offboard, float current_relative_altitude_m);
};
