#ifndef FLY_MISSION_HPP
#define FLY_MISSION_HPP

#include "async_mqtt.hpp"
#include "mqtt_client.hpp"
#include <atomic>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

int fly_mission(const std::string &missionId, Mavsdk_members &mavsdk, const std::string &waypoint_route);

#endif // FLY_MISSION_HPP