#ifndef TARGET_TRACKER_HPP
#define TARGET_TRACKER_HPP

#include "async_mqtt.hpp"
#include <atomic>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

extern std::atomic<bool> detection_running;

extern double last_speed_x;
extern double last_speed_y;
extern double err_x;
extern double err_y;

void detectLandingPadAndSendCommand(Mavsdk_members &mavsdk);

#endif
