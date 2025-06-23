#include "telemetry_monitor.hpp"
#include "logger.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

using namespace mavsdk;

TelemetryMonitor::TelemetryMonitor(Telemetry &telemetry) : telemetry(telemetry)
{
    monitor_thread_ = std::thread(&TelemetryMonitor::monitorLoop, this); // 启动监控线程
    land_threshold_ = 0.5f;                                              // 设置降落判定阈值为0.5米
    current_relative_altitude_m = 0.0f;                                  // 初始化当前相对高度
    has_landed.store(false);                                             // 初始化降落状态为未降落
    running_.store(true);                                                // 设置线程运行标志为true
}

TelemetryMonitor::~TelemetryMonitor()
{
    if (running_.exchange(false))
    {
        if (monitor_thread_.joinable())
        {
            monitor_thread_.join();
        }
    }
}

void TelemetryMonitor::monitorLoop()
{
    // 订阅位置数据
    auto position_handle = telemetry.subscribe_position(
        [this](Telemetry::Position position)
        {
            std::lock_guard<std::mutex> lock(altitude_mutex_);
            current_relative_altitude_m = position.relative_altitude_m; // 更新当前相对高度(单位:米)

            if (position.relative_altitude_m < land_threshold_)
            {
                has_landed.store(true);
            }
        });

    // 订阅位置-速度数据
    auto position_velocity_handle = telemetry.subscribe_position_velocity_ned(
        [this](Telemetry::PositionVelocityNed position_velocity_ned)
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            current_position = position_velocity_ned.position; // 更新当前无人机位置(NED坐标系)
        });

    // 循环等待直到停止或检测到着陆
    while (running_.load() && !has_landed.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 清理所有订阅
    telemetry.unsubscribe_position(position_handle);
    telemetry.unsubscribe_position_velocity_ned(position_velocity_handle);
}

// 获取当前位置
Telemetry::PositionNed TelemetryMonitor::getCurrentPosition() const
{
    std::lock_guard<std::mutex> lock(position_mutex_);
    return current_position;
}

// 获取当前相对高度
float TelemetryMonitor::getCurrentRelativeAltitudeM() const
{
    std::lock_guard<std::mutex> lock(altitude_mutex_);
    return current_relative_altitude_m;
}

// 获取着陆状态
bool TelemetryMonitor::hasLanded() const
{
    return has_landed.load();
}

// 设置着陆完成标志
void TelemetryMonitor::setHasLanded()
{
    has_landed.store(true);
}

// 设置飞行模式回调
void TelemetryMonitor::set_flight_mode_callback(FlightModeCallback callback)
{
    std::lock_guard<std::mutex> lock(flight_mode_mutex_);

    // 取消之前的订阅并设置新的回调
    flight_mode_callback_ = std::move(callback);

    if (flight_mode_callback_)
    {
        flight_mode_handle_.reset(new Telemetry::FlightModeHandle(
            telemetry.subscribe_flight_mode(
                [this](Telemetry::FlightMode flight_mode)
                {
                    std::lock_guard<std::mutex> lock(flight_mode_mutex_);
                    if (flight_mode_callback_)
                    {
                        flight_mode_callback_(flight_mode);
                    }
                })));
    }
    else
    {
        // 如果回调为空，则取消订阅
        if (flight_mode_handle_)
        {
            telemetry.unsubscribe_flight_mode(*flight_mode_handle_);
            flight_mode_handle_.reset();
        }
    }
}