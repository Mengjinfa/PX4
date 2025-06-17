#include "detection_state_machine.hpp"
#include <chrono>

DetectionStateMachine::DetectionStateMachine() = default;

void DetectionStateMachine::setCurrentRelativeAltitude(float altitude)
{
    in_altitude = altitude;
}

float DetectionStateMachine::getNotDetectedAltitude()
{
    return saved_altitude;
}

DroneState DetectionStateMachine::update(bool detected)
{
    auto now = std::chrono::steady_clock::now();

    switch (state)
    {
        case DroneState::SEARCHING:
            if (detected)
            {
                detection_count++;
                if (detection_count >= (has_tracked_once ? 1 : 30))
                {
                    state = DroneState::TRACKING;
                    detection_count = 0; // 重置计数器
                    no_detection_count = 0;
                    has_tracked_once = true; // 标记已进入过 TRACKING
                }
            }
            else
            {
                auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - search_start).count();
                if (duration >= 5)
                {
                    state = DroneState::NOT_DETECTED;
                    no_detection_count = 0;
                    detection_count = 0;
                    saved_altitude = in_altitude;
                }
            }
            break;

        case DroneState::TRACKING:
            if (detected)
            {
                no_detection_count = 0; // 重置未检测计数
            }
            else
            {
                no_detection_count++;
                if (no_detection_count >= 10)
                {
                    state = DroneState::SEARCHING;
                    search_start = std::chrono::steady_clock::now();
                    no_detection_count = 0; // 重置计数器
                    detection_count = 0;    // 重置识别计数
                    saved_altitude = in_altitude;
                }
            }
            break;

        case DroneState::NOT_DETECTED:
            if (detected)
            {
                state = DroneState::SEARCHING;
                search_start = std::chrono::steady_clock::now();
                saved_altitude = in_altitude;
            }
            break;
    }

    return state;
}
bool DetectionStateMachine::isLandingComplete() const
{
    return landing_complete.load();
}