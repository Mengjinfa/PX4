#include "pid.hpp"

/**
 * @brief PID控制器构造函数
 * 初始化控制器参数和状态
 */
PID::PID() : last_landmark_recordcnt_(0.0), is_first_detection_(true), current_step_(0)
{
    initialize();
    last_landmark_recordcnt_ = get_current_time();
}

/**
 * @brief 初始化控制器
 * 设置初始参数和状态
 */
void PID::initialize()
{
    // 初始化PID参数
    pid_params_.kp = 0.002;
    pid_params_.ki = 0.0;
    pid_params_.kd = 0.0001;

    // 初始化误差状态
    error_x_ = {0};
    error_y_ = {0};

    // 初始化控制输出
    pid_output_ = {0};
    pid_output_.timestamp = get_current_time();

    // 初始化状态变量
    is_first_detection_ = true;
    current_step_ = 0;
}

/**
 * @brief 获取当前检测到的地标位置
 * @param data 包含地标位置信息的结构体
 */
void PID::getLandmark(const AprilTagData &data)
{
    // 检查地标是否有效
    if (data.iffind)
    {
        last_landmark_ = landmark_;
        landmark_ = data;
    }
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

    preprocess_Landmark();  // 预处理地标数据
    First_Detection();      // 处理首次检测到地标
    apply_LowPass_Filter(); // 应用低通滤波
    calculate_PID(dt);      // 计算PID控制量
}

/**
 * @brief 预处理地标数据
 * 处理地标无效情况，计算误差
 */
void PID::preprocess_Landmark()
{
    if (landmark_.x == 0 && landmark_.y == 0 && (last_landmark_.x != 0 || last_landmark_.y != 0))
    {
        // 两次地标信息更新时间超过0.5s，认为地标丢失，将地标信息置为0
        if ((get_current_time() - last_landmark_recordcnt_) > 0.5)
        {
            last_landmark_.x = 0.0;
            last_landmark_.y = 0.0;
            is_first_detection_ = true;
        }
    }
    else // 保存有效地标位置
    {
        last_landmark_.x = landmark_.x;
        last_landmark_.y = landmark_.y;
        last_landmark_recordcnt_ = get_current_time();
    }

    // 处理地标无效情况，使用上一有效位置
    double landmark_x = (landmark_.x == 0) ? (last_landmark_.x > 0 ? last_landmark_.x : landmark_.x) : landmark_.x;
    double landmark_y = (landmark_.y == 0) ? (last_landmark_.y > 0 ? last_landmark_.y : landmark_.y) : landmark_.y;

    // 计算图像中心与地标中心的误差
    error_x_.err = (landmark_.width / 2.0) - landmark_x;
    error_y_.err = (landmark_.height / 2.0) - landmark_y;
}

/**
 * @brief 处理首次检测到地标
 * 平滑过渡控制响应，避免突变
 */
void PID::First_Detection()
{
    if (is_first_detection_)
    {
        if (landmark_.x != 0 && landmark_.y != 0)
        {
            // 首次检测到地标，使用渐进式控制避免突变
            double ramp_factor = std::min(1.0, static_cast<double>(current_step_) / 100.0);
            error_x_.current = error_x_.err * ramp_factor;
            error_y_.current = error_y_.err * ramp_factor;

            current_step_++;
            if (current_step_ >= 100)
            {
                is_first_detection_ = false;
                current_step_ = 0;
            }
        }
    }
    else
    {
        error_x_.current = error_x_.err;
        error_y_.current = error_y_.err;
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

    // 应用低通滤波器，减少高频噪声
    error_x_.filtered = alpha * error_x_.err + (1 - alpha) * error_x_.last_filtered;
    error_y_.filtered = alpha * error_y_.err + (1 - alpha) * error_y_.last_filtered;
}

/**
 * @brief 计算PID控制量
 * 根据误差计算速度控制指令
 */
void PID::calculate_PID(double dt)
{
    // 保存上一时刻误差
    error_x_.last = error_x_.err;
    error_y_.last = error_y_.err;

    // 积分项累积
    error_x_.integral += error_x_.err * dt;
    error_y_.integral += error_y_.err * dt;

    // 限制积分项大小，防止积分饱和
    error_x_.integral = std::max(std::min(error_x_.integral, 100.0), -100.0);
    error_y_.integral = std::max(std::min(error_y_.integral, 100.0), -100.0);

    // 微分项计算（使用滤波后的值）
    error_x_.derivative = (error_x_.filtered - error_x_.last_filtered) / dt;
    error_y_.derivative = (error_y_.filtered - error_y_.last_filtered) / dt;

    // 计算最终控制输出
    pid_output_.x = pid_params_.kp * error_x_.filtered + pid_params_.ki * error_x_.integral + pid_params_.kd * error_x_.derivative;
    pid_output_.y = pid_params_.kp * error_y_.filtered + pid_params_.ki * error_y_.integral + pid_params_.kd * error_y_.derivative;
}

/**
 * @brief 设置PID参数
 */
void PID::setPIDParameters(const PIDParameters &params)
{
    pid_params_ = params;
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