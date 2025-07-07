#include "telemetry_monitor.hpp"
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>

using namespace mavsdk;

TelemetryMonitor::TelemetryMonitor(Telemetry &telemetry) : telemetry(telemetry)
{
    monitor_thread_ = std::thread(&TelemetryMonitor::monitorLoop, this); // 启动监控线程

    // 初始化监控器状态
    running_.store(true);

    current_distance_sensor_m = std::numeric_limits<float>::min(); // 初始化距离传感器高度为最小值
    current_flight_mode_ = Telemetry::FlightMode::Unknown;         // 初始化飞行模式为未知
    current_position = Telemetry::PositionNed{};                   // 初始化当前位置为默认值
    current_gps_ = Telemetry::RawGps{};                            // 初始化GPS信息为默认值
    current_relative_altitude_m = 0.0f;                            // 初始化相对高度为0
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

// 飞机数据监控线程主循环
void TelemetryMonitor::monitorLoop()
{
    // 订阅高度数据
    telemetry.subscribe_position(
        [this](Telemetry::Position position)
        {
            std::lock_guard<std::mutex> lock(altitude_mutex_);
            current_relative_altitude_m = position.relative_altitude_m;
        });

    // 订阅位置数据
    telemetry.subscribe_position_velocity_ned(
        [this](Telemetry::PositionVelocityNed position_velocity_ned)
        {
            std::lock_guard<std::mutex> lock(position_mutex_);
            current_position = position_velocity_ned.position;
        });

    // 订阅飞行模式数据
    std::make_unique<Telemetry::FlightModeHandle>(
        telemetry.subscribe_flight_mode(
            [this](Telemetry::FlightMode flight_mode)
            {
                std::lock_guard<std::mutex> lock(flight_mode_mutex_);
                current_flight_mode_ = flight_mode;
            }));

    // 订阅GPS信息
    std::make_unique<Telemetry::RawGpsHandle>(
        telemetry.subscribe_raw_gps(
            [this](Telemetry::RawGps gps_raw)
            {
                std::lock_guard<std::mutex> lock(gps_mutex_);
                current_gps_ = gps_raw;
            }));

    // 订阅距离传感器数据
    std::make_unique<Telemetry::DistanceSensorHandle>(
        telemetry.subscribe_distance_sensor(
            [this](Telemetry::DistanceSensor distance_sensor)
            {
                std::lock_guard<std::mutex> lock(distance_mutex_);
                current_distance_sensor_m = distance_sensor.current_distance_m;
            }));

    // 订阅欧拉角数据
    std::make_unique<Telemetry::AttitudeEulerHandle>(
        telemetry.subscribe_attitude_euler(
            [this](Telemetry::EulerAngle attitude_euler)
            {
                std::lock_guard<std::mutex> lock(euler_angle_mutex_);
                m_euler_angle = attitude_euler;
            }));

    // 循环等待直到停止或检测到着陆
    while (running_.load())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// 获取当前相对高度
float TelemetryMonitor::getCurrentRelativeAltitudeM() const
{
    std::lock_guard<std::mutex> lock(altitude_mutex_);
    return current_relative_altitude_m;
}

// 获取当前位置
Telemetry::PositionNed TelemetryMonitor::getCurrentPosition() const
{
    std::lock_guard<std::mutex> lock(position_mutex_);
    return current_position;
}

// 获取当前飞行模式
Telemetry::FlightMode TelemetryMonitor::getCurrentFlightMode() const
{
    std::lock_guard<std::mutex> lock(flight_mode_mutex_);
    return current_flight_mode_;
}

// 获取当前GPS信息
Telemetry::RawGps TelemetryMonitor::getCurrentRawGps() const
{
    std::lock_guard<std::mutex> lock(gps_mutex_);
    return current_gps_;
}

// 获取当前距离传感器高度
float TelemetryMonitor::getCurrentDistanceSensorM() const
{
    std::lock_guard<std::mutex> lock(distance_mutex_);
    return current_distance_sensor_m;
}

// 获取当前欧拉角姿态
Telemetry::EulerAngle TelemetryMonitor::getCurrentEulerAngles() const
{
    std::lock_guard<std::mutex> lock(euler_angle_mutex_);
    return m_euler_angle;
}

// 辅助函数：将飞行模式枚举转换为字符串
std::string TelemetryMonitor::flight_mode_str(mavsdk::Telemetry::FlightMode mode)
{
    switch (mode)
    {
        case Telemetry::FlightMode::Unknown:
            return "Unknown";
        case Telemetry::FlightMode::Ready:
            return "Ready";
        case Telemetry::FlightMode::Takeoff:
            return "Takeoff";
        case Telemetry::FlightMode::Hold:
            return "Hold";
        case Telemetry::FlightMode::Mission:
            return "Mission";
        case Telemetry::FlightMode::ReturnToLaunch:
            return "ReturnToLaunch";
        case Telemetry::FlightMode::Land:
            return "Land";
        case Telemetry::FlightMode::Offboard:
            return "Offboard";
        case Telemetry::FlightMode::FollowMe:
            return "FollowMe";
        case Telemetry::FlightMode::Posctl:
            return "Position";
        case Telemetry::FlightMode::Altctl:
            return "Altitude";
        case Telemetry::FlightMode::Stabilized:
            return "Stabilized";
        case Telemetry::FlightMode::Acro:
            return "Acro";
        default:
            return "Invalid";
    }
}