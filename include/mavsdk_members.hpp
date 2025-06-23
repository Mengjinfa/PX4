#ifndef MAVSDK_MEMBERS_HPP
#define MAVSDK_MEMBERS_HPP

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

class Mavsdk_members
{
public:
    Mavsdk_members(mavsdk::MavlinkPassthrough &mp,
                   mavsdk::MissionRaw &mr,
                   mavsdk::Telemetry &t,
                   mavsdk::Offboard &o,
                   mavsdk::Mission &m,
                   mavsdk::Action &a,
                   mavsdk::Camera &c);

    mavsdk::MavlinkPassthrough &mavlink_passthrough;
    mavsdk::MissionRaw &mission_raw;
    mavsdk::Telemetry &telemetry;
    mavsdk::Offboard &offboard;
    mavsdk::Mission &mission;
    mavsdk::Action &action;
    mavsdk::Camera &camera;
};

#endif