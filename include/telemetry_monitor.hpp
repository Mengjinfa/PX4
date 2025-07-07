#pragma once

#include "mavsdk_members.hpp"

#include <atomic>
#include <functional>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <mutex>
#include <thread>

#include "singleton.hpp"

namespace mavsdk
{
    class Telemetry;
}

/**
 * @brief 无人机遥测数据监控器，用于实时跟踪无人机状态
 *
 * 该类通过独立线程持续获取无人机位置、高度等信息，并提供线程安全的接口供其他模块查询当前状态。
 */
class TelemetryMonitor
{
public:
    TelemetryMonitor(mavsdk::Telemetry &telemetry);
    ~TelemetryMonitor(); // 析构函数，确保监控线程正确停止

    float getCurrentRelativeAltitudeM() const; // 获取当前相对起飞点的高度
    float getCurrentDistanceSensorM() const;   // 获取当前距离传感器高度

    mavsdk::Telemetry::PositionNed getCurrentPosition() const;   // 获取当前无人机位置(NED坐标系)
    mavsdk::Telemetry::FlightMode getCurrentFlightMode() const;  // 获取当前飞行模式
    mavsdk::Telemetry::RawGps getCurrentRawGps() const;          // 获取GPS信息
    mavsdk::Telemetry::EulerAngle getCurrentEulerAngles() const; // 获取当前欧拉角姿态

    std::string flight_mode_str(mavsdk::Telemetry::FlightMode mode); // 将飞行模式枚举转换为字符串

private:
    void monitorLoop(); // 监控线程主循环，持续更新无人机状态

    mavsdk::Telemetry &telemetry;                       // 外部传入的遥测插件引用，用于获取无人机数据
    mavsdk::Telemetry::PositionNed current_position;    // 当前无人机位置(NED坐标)
    mavsdk::Telemetry::FlightMode current_flight_mode_; // 存储当前飞行模式
    mavsdk::Telemetry::RawGps current_gps_;             // 存储当前GPS信息
    mavsdk::Telemetry::EulerAngle m_euler_angle;        // 存储当前欧拉角姿态

    float land_threshold_;             // 降落判定阈值(当相对高度低于此值时判定为已降落)
    float current_relative_altitude_m; // 当前相对高度(单位:米)
    float current_distance_sensor_m;   // 当前距离传感器高度(单位:米)

    mutable std::mutex position_mutex_;    // 保护位置数据的互斥锁
    mutable std::mutex altitude_mutex_;    // 保护高度数据的互斥锁
    mutable std::mutex flight_mode_mutex_; // 保护飞行模式的互斥锁
    mutable std::mutex gps_mutex_;         // 保护GPS数据的互斥锁
    mutable std::mutex distance_mutex_;    // 保护距离传感器数据的互斥锁
    mutable std::mutex euler_angle_mutex_; // 保护欧拉角数据的互斥锁

    std::atomic<bool> running_{true}; // 线程运行标志(原子操作确保多线程安全)
    std::thread monitor_thread_;      // 运行monitorLoop()的独立线程
};

// typedef NormalSingleton<TelemetryMonitor> telemetry_monitor;