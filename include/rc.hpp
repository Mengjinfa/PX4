#ifndef RC_HPP
#define RC_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>


int rc_control(mavsdk::MavlinkPassthrough& mavlink_passthrough,mavsdk::Action& action ,mavsdk::Offboard& offboard,mavsdk::Telemetry& telemetry);

#endif 