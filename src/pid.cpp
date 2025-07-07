#include "pid.hpp"

/**
 * @brief PID控制器构造函数
 * 初始化控制器参数和状态
 */
PID::PID() : is_first_detection_(true), current_step_(0)
{
    // 初始化PID参数
    pid_params_.kp = 0.002;
    pid_params_.ki = 0.0;
    pid_params_.kd = 0.0005;

    // 初始化误差状态
    error_x_ = {};
    error_y_ = {};

    // 初始化控制输出
    pid_output_.x = 0;
    pid_output_.y = 0;
    pid_output_.timestamp = get_current_time();

    // 初始化状态变量
    is_first_detection_ = true;
    current_step_ = 0;
}

/**
 * @brief 计算控制指令
 * 执行完整的控制计算流程
 */
void PID::PID_update()
{
    double now = get_current_time();
    double dt = now - pid_output_.timestamp;
    if (dt <= 0.0)
    {
        dt = 0.001; // 防止除零错误
    }
    pid_output_.timestamp = now;

    First_Detection();      // 处理首次检测
    apply_LowPass_Filter(); // 应用低通滤波
    calculate_PID(dt);      // 计算PID控制量
}

/**
 * @brief 处理首次检测
 * 平滑过渡控制响应，避免突变
 */
void PID::First_Detection()
{
    if (is_first_detection_)
    {
        // 首次检测，使用渐进式控制避免突变
        double ramp_factor = std::min(1.0, static_cast<double>(current_step_) / 100.0);
        error_x_.current = landmark_.err_x * ramp_factor;
        error_y_.current = landmark_.err_y * ramp_factor;

        current_step_++;
        if (current_step_ >= 100)
        {
            is_first_detection_ = false;
            current_step_ = 0;
        }
    }
    else
    {
        error_x_.current = landmark_.err_x;
        error_y_.current = landmark_.err_y;
    }
}

/**
 * @brief 应用低通滤波
 * 减少噪声影响，平滑控制响应
 */
void PID::apply_LowPass_Filter()
{
    static double alpha = 0.2; // 低通滤波系数

    // 保存上一时刻滤波值
    error_x_.last_filtered = error_x_.filtered;
    error_y_.last_filtered = error_y_.filtered;

    // 应用低通滤波器
    error_x_.filtered = alpha * error_x_.current + (1 - alpha) * error_x_.last_filtered;
    error_y_.filtered = alpha * error_y_.current + (1 - alpha) * error_y_.last_filtered;
}

/**
 * @brief 计算PID控制量
 * 根据误差计算控制指令
 */
void PID::calculate_PID(double dt)
{
    // 积分项累积
    error_x_.integral += error_x_.filtered * dt;
    error_y_.integral += error_y_.filtered * dt;

    // 限制积分项大小，防止积分饱和
    error_x_.integral = std::max(std::min(error_x_.integral, 100.0), -100.0);
    error_y_.integral = std::max(std::min(error_y_.integral, 100.0), -100.0);

    // 微分项计算（使用滤波后的值）
    error_x_.derivative = (error_x_.filtered - error_x_.last_filtered) / dt;
    error_y_.derivative = (error_y_.filtered - error_y_.last_filtered) / dt;

    // 计算最终控制输出
    pid_output_.x = pid_params_.kp * error_x_.filtered +
                    pid_params_.ki * error_x_.integral +
                    pid_params_.kd * error_x_.derivative;

    pid_output_.y = pid_params_.kp * error_y_.filtered +
                    pid_params_.ki * error_y_.integral +
                    pid_params_.kd * error_y_.derivative;
}

/**
 * @brief 获取当前检测到的地标位置
 * @param data 包含地标位置信息的结构体
 */
void PID::getLandmark(const AprilTagData &data)
{
    landmark_ = data;
}

/**
 * @brief 输出PID控制结果
 */
PIDOutput PID::Output_PID() const
{
    return pid_output_;
}

/**
 * @brief 获取当前时间(秒)
 */
double PID::get_current_time() const
{
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double>(duration).count();
}