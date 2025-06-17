#pragma once
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include "target_tracker.hpp"
#include <chrono>

struct PIDController {
    double kp, ki, kd;
    double integral;
    double last_error;

    PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), integral(0.0), last_error(0.0) {}

    double compute(double error, double dt) {
    // 先计算 derivative，避免未定义行为
    double derivative = 0.0;
    if (dt > 0) {
        derivative = (error - last_error) / dt;
        integral += error * dt;

        // 防止积分饱和
        constexpr double integral_min = -1.0;
        constexpr double integral_max = 1.0;
        integral = std::clamp(integral, integral_min, integral_max);
    }

    last_error = error;
    return kp * error + ki * integral + kd * derivative;
    }

    void reset() {
        integral = 0.0;
        last_error = 0.0;
    }
};

class DroneController {
public:
    explicit DroneController(float max_speed, float radius)
        : MAX_SPEED(max_speed),
          RADIUS(radius),
          last_time(std::chrono::steady_clock::now()) {
        // 确保 alpha 合法
        alpha = std::clamp(alpha, 0.0, 1.0);
    }

    void trackTarget(double dx, double dy, mavsdk::Offboard& offboard, const mavsdk::Telemetry::PositionNed& current, float current_relative_altitude_m);
    void searchPattern(double elapsed, const mavsdk::Telemetry::PositionNed& current, mavsdk::Offboard& offboard,float current_relative_altitude_m);

private:
    static constexpr float MAX_SPEED_DEFAULT = 0.2f;
    static constexpr float RADIUS_DEFAULT = 0.5f;
    static constexpr double ANGULAR_VELOCITY_DEFAULT = 0.2;
    static constexpr double TRANSITION_TIME_DEFAULT = 5.0;
    static constexpr double START_ANGLE_DEFAULT = 0.0;

    const float MAX_SPEED;
    const float RADIUS;
    const double ANGULAR_VELOCITY = ANGULAR_VELOCITY_DEFAULT;
    const double transition_time = TRANSITION_TIME_DEFAULT;
    const double start_angle = START_ANGLE_DEFAULT;

    PIDController pid_forward{0.5, 0.1, 0.01}; // 前后方向
    PIDController pid_right{0.5, 0.1, 0.01};   // 左右方向
    PIDController pid_down{0.3, 0.05, 0.005};  //上下方向


    // PIDController pid_forward{0.02, 0.0,  0.0001}; // 前后方向
    // PIDController pid_right{0.02, 0.0,  0.0001};   // 左右方向
    // PIDController pid_down{0.02, 0.0,  0.0001};  //上下方向

    double alpha = 0.2; // 滤波系数，越小越平滑

    // 上次时间戳
    std::chrono::steady_clock::time_point last_time;

    // 滤波后的值
    double filtered_dx = 0.0;
    double filtered_dy = 0.0;

    // 滤波函数
    double applyFilter(double raw_value, double& filtered_value) {
        return alpha * raw_value + (1.0 - alpha) * filtered_value;
    }
};















































// #pragma once
// #include <mavsdk/plugins/offboard/offboard.h>
// #include <mavsdk/plugins/telemetry/telemetry.h>
// #include "start_detect_target.hpp"
// #include <chrono>


// struct PIDController {
//     double kp, ki, kd;
//     double integral;
//     double last_error;

//     PIDController(double kp, double ki, double kd) : kp(kp), ki(ki), kd(kd), integral(0.0), last_error(0.0) {}

//     double compute(double error, double dt) {
//         integral += error * dt;
//         double derivative = (error - last_error) / dt;
//         last_error = error;
//         return kp * error + ki * integral + kd * derivative;
//     }

//     void reset() {
//         integral = 0.0;
//         last_error = 0.0;
//     }
// };





// class DroneController {
// public:
//     explicit DroneController(float max_speed, float radius);

//     void trackTarget(double dx, double dy, mavsdk::Offboard& offboard, const mavsdk::Telemetry::PositionNed& current,float current_relative_altitude_m);
//     void searchPattern(double elapsed, const mavsdk::Telemetry::PositionNed& current, mavsdk::Offboard& offboard);

// private:
//     float MAX_SPEED = 0.2;
//     float RADIUS = 0.5;
//     double ANGULAR_VELOCITY = 0.2;
//     double transition_time = 5.0;
//     double start_angle = 0.0;

//     PIDController pid_forward{0.5, 0.1, 0.01}; // 前后方向
//     PIDController pid_right{0.5, 0.1, 0.01};   // 左右方向

//     double alpha = 0.3; // 滤波系数，越小越平滑

//     // 上次时间戳
//     std::chrono::steady_clock::time_point last_time;

//     // 滤波后的值
//     double filtered_dx = 0.0;
//     double filtered_dy = 0.0;
    
// };