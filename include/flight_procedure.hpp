#ifndef FLIGHT_PROCEDURE_HPP
#define FLIGHT_PROCEDURE_HPP

#include "mavsdk_members.hpp"

int offboard_flight_position(Mavsdk_members &mavsdk, float north_m, float east_m, float down_m, float yaw_deg);
int offboard_flight_body_velocity(Mavsdk_members &mavsdk, float forward_m_s, float right_m_s, float down_m_s, float yaw_rate_deg_s);

int arming_and_takeoff(Mavsdk_members &mavsdk, float takeoff_altitude_m);
int land_and_disarm(Mavsdk_members &mavsdk);

#endif // FLIGHT_PROCEDURE_HPP