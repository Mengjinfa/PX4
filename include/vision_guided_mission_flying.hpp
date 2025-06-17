#ifndef FLY_MISSION_HPP
#define FLY_MISSION_HPP

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <atomic>



extern std::atomic<int> start_detect;
extern std::atomic<int> find_landmark;
extern std::atomic<int> start_posctl;
extern std::atomic<int> is_take_off;


int fly_mission(const std::string& missionId,mavsdk::Action& action ,mavsdk::Mission& mission,mavsdk::Telemetry& telemetry , mavsdk::Offboard& offboard,mavsdk::Camera& camera,mavsdk::MavlinkPassthrough& mavlink_passthrough);

#endif // FLY_MISSION_HPP