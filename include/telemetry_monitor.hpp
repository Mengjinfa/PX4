#pragma once
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <atomic>
#include <thread>
#include <mutex>

class TelemetryMonitor {
public:
    explicit TelemetryMonitor(mavsdk::Telemetry& telemetry, float land_threshold = 0.2f);
    ~TelemetryMonitor();


    bool hasLanded() const;
    void setHasLanded();
    const mavsdk::Telemetry::PositionNed getCurrentPosition() const;
    const float getCurrentRelativeAltitudeM() const;
private:
    void monitorLoop();

    mavsdk::Telemetry& telemetry;
    float land_threshold_;
    std::atomic<bool> has_landed{false};
    mavsdk::Telemetry::PositionNed current_position;
    float current_relative_altitude_m;
    mutable std::mutex position_mutex_;
    mutable std::mutex altitude_mutex_;
    std::thread monitor_thread_;
};