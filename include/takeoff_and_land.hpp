#ifndef TAKEOFF_AND_LAND_HPP
#define TAKEOFF_AND_LAND_HPP

#include "async_mqtt.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

int takeoff_and_land(int enable, Mavsdk_members &mavsdk);

#endif // TAKEOFF_AND_LAND_HPP