#ifndef FLIGHT_PROCEDURE_HPP
#define FLIGHT_PROCEDURE_HPP

#include "mavsdk_members.hpp"
#include <chrono>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

// 明确使用mavsdk命名空间
using namespace mavsdk;

// 函数声明
bool offboard_flight_position(Mavsdk_members &mavsdk, float north_m, float east_m, float down_m, float yaw_deg);
bool offboard_flight_body_velocity(Mavsdk_members &mavsdk, float forward_m_s, float right_m_s, float down_m_s, float yaw_rate_deg_s);

int arming_and_takeoff(Mavsdk_members &mavsdk, float takeoff_altitude_m);
int land_and_disarm(Mavsdk_members &mavsdk);

#endif // FLIGHT_PROCEDURE_HPP