#ifndef START_DETECT_TARGET
#define START_DETECT_TARGET

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <atomic>

// void detectLandingPadAndSendCommand(mavsdk::MavlinkPassthrough& mavlink_passthrough);
// void send_offboard_velocity(mavsdk::MavlinkPassthrough& mavlink_passthrough, float vx, float vy);

extern std::atomic<bool> detection_running;

extern double last_speed_x;
extern double last_speed_y;
extern double err_x;
extern double err_y;

void detectLandingPadAndSendCommand(mavsdk::Offboard& offboard, mavsdk::Telemetry& telemetry,mavsdk::MavlinkPassthrough& mavlink_passthrough);

#endif 