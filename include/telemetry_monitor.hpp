#pragma once
#include <atomic>
#include <functional>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <memory>
#include <mutex>
#include <thread>

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

    bool hasLanded() const;                                    // 判断无人机是否已降落
    void setHasLanded();                                       // 手动设置降落状态(用于特殊场景覆盖自动判定)
    float getCurrentRelativeAltitudeM() const;                 // 获取当前相对起飞点的高度
    mavsdk::Telemetry::PositionNed getCurrentPosition() const; // 获取当前无人机位置(NED坐标系)

    // 飞行模式回调类型定义和设置函数
    using FlightModeCallback = std::function<void(mavsdk::Telemetry::FlightMode)>;
    void set_flight_mode_callback(FlightModeCallback callback);

private:
    void monitorLoop(); // 监控线程主循环，持续更新无人机状态

    mavsdk::Telemetry &telemetry;                    // 外部传入的遥测插件引用，用于获取无人机数据
    mavsdk::Telemetry::PositionNed current_position; // 当前无人机位置(NED坐标)

    float land_threshold_;              // 降落判定阈值(当相对高度低于此值时判定为已降落)
    float current_relative_altitude_m;  // 当前相对高度(单位:米)
    mutable std::mutex position_mutex_; // 保护位置数据的互斥锁
    mutable std::mutex altitude_mutex_; // 保护高度数据的互斥锁

    std::atomic<bool> has_landed{false}; // 降落状态标志(原子操作确保多线程安全)
    std::atomic<bool> running_{true};    // 线程运行标志
    std::thread monitor_thread_;         // 运行monitorLoop()的独立线程

    // 飞行模式监控相关成员
    mutable std::mutex flight_mode_mutex_;
    FlightModeCallback flight_mode_callback_;
    std::unique_ptr<mavsdk::Telemetry::FlightModeHandle> flight_mode_handle_;
};