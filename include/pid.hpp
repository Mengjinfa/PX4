#ifndef PID_HPP
#define PID_HPP

#include "apriltag_tracker.hpp"
#include "singleton.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>

// PID输出结构
struct PIDOutput
{
    double x;         // x方向控制输出
    double y;         // y方向控制输出
    double timestamp; // 时间戳(秒)
};

// PID控制器类
class PID
{

public:
    PID(); // 默认构造函数

    void getLandmark(const AprilTagData &data); // 获取地标数据
    void PID_update();                          // 计算PID控制指令
    PIDOutput Output_PID() const;               // 输出PID控制结果

private:
    // PID参数结构
    struct PIDParameters
    {
        double kp; // 比例系数
        double ki; // 积分系数
        double kd; // 微分系数
    };

    // 误差状态结构
    struct ErrorState
    {
        double current;       // 当前处理后误差
        double integral;      // 积分项
        double derivative;    // 微分项
        double filtered;      // 滤波后误差
        double last_filtered; // 上一时刻滤波后误差
    };

    // 私有成员变量
    PIDParameters pid_params_;   // PID控制参数
    AprilTagData landmark_;      // 当前地标数据
    AprilTagData last_landmark_; // 上一帧地标数据
    ErrorState error_x_;         // x方向误差状态
    ErrorState error_y_;         // y方向误差状态
    PIDOutput pid_output_;       // PID控制输出
    bool is_first_detection_;    // 是否首次检测到地标
    int current_step_;           // 渐进控制步数

    // 获取当前时间(秒)
    double get_current_time() const;

private:
    void First_Detection();        // 处理首次检测到地标的情况
    void apply_LowPass_Filter();   // 应用低通滤波器
    void calculate_PID(double dt); // 计算PID控制量
};

typedef NormalSingleton<PID> pid;

#endif // PID_HPP