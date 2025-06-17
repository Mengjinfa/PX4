#include "telemetry_monitor.hpp"
#include <chrono>

// 引入全局日志模块
#include "logger.hpp"

using namespace mavsdk;

TelemetryMonitor::TelemetryMonitor(Telemetry& telemetry_, float land_threshold)
    : telemetry(telemetry_), land_threshold_(land_threshold) {
    monitor_thread_ = std::thread(&TelemetryMonitor::monitorLoop, this);

    // 确保线程可分离或在析构中 join
    if (monitor_thread_.joinable()) {
        monitor_thread_.detach(); // 或者保存 thread 并在析构中 join
    }
}

TelemetryMonitor::~TelemetryMonitor() {
    if (monitor_thread_.joinable()) {
        monitor_thread_.join();
    }
}

bool TelemetryMonitor::hasLanded() const {
    return has_landed.load();
}

void TelemetryMonitor::setHasLanded() {
    has_landed.store(true);
}

const Telemetry::PositionNed TelemetryMonitor::getCurrentPosition() const {
    std::lock_guard<std::mutex> lock(position_mutex_);
    return current_position;
}

const float TelemetryMonitor::getCurrentRelativeAltitudeM() const {
    std::lock_guard<std::mutex> lock(altitude_mutex_);
    return current_relative_altitude_m;
}

void TelemetryMonitor::monitorLoop() {
    auto position_handle = telemetry.subscribe_position([this](Telemetry::Position position) {
        this->current_relative_altitude_m = position.relative_altitude_m;
        if (position.relative_altitude_m < land_threshold_) {
            has_landed.store(true);
            logging::get_logger()->info("当前高度zxkkkk111111：{}", position.relative_altitude_m);
        }
    });

    bool is_get_position = false;

    auto position_velocity_ned_handle = telemetry.subscribe_position_velocity_ned(
        [this, &is_get_position](Telemetry::PositionVelocityNed position_velocity_ned) {
            this->current_position = position_velocity_ned.position;
            is_get_position = true;
        });
    if (false) {
    // if (!position_velocity_ned_handle) {
        logging::get_logger()->error("Failed to subscribe to position velocity NED data.");
    } else {
        // 可选：等待首次获取位置数据
        // const auto timeout = std::chrono::seconds(10);
        // const auto start_time = std::chrono::steady_clock::now();
        // while (!is_get_position) {
        //     std::this_thread::sleep_for(std::chrono::milliseconds(20));
        //     if (std::chrono::steady_clock::now() - start_time > timeout) {
        //         logging::get_logger()->warn("Timeout waiting for position data.");
        //         break;
        //     }
        // }

        // telemetry.unsubscribe_position_velocity_ned(position_velocity_ned_handle);
    }

    // 持续运行直到对象销毁
    while (!has_landed.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    logging::get_logger()->info("monitor的值2222::{}", has_landed.load());
    logging::get_logger()->info("开始结束monitorLoop");

    telemetry.unsubscribe_position(position_handle);
    telemetry.unsubscribe_position_velocity_ned(position_velocity_ned_handle);

    logging::get_logger()->info("开始结束122333monitorLoop");
}