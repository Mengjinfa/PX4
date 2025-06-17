#ifndef TAKEOFF_AND_LAND_HPP
#define TAKEOFF_AND_LAND_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>



int takeoff_and_land(int enable,mavsdk::Action& action,mavsdk::Telemetry& telemetry,mavsdk::Offboard& offboard);

#endif // TAKEOFF_AND_LAND_HPP