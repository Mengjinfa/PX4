#ifndef USER_TASK_HPP
#define USER_TASK_HPP

#include "flight_procedure.hpp"
#include "mavsdk_members.hpp"
#include "mqtt_client.hpp"

#include <chrono>
#include <cmath>
#include <string>

// PID输出结构
struct UserTask
{
    bool takeoff_task_flag = false;
    bool landing_task_flag = false;
    bool waypoint_task_flag = false;
    bool land_mode_flag = false;
};
extern UserTask user_task;

void handleTestMessage(const std::vector<unsigned char> &payload);
void userTaskProcedure(Mavsdk_members &mavsdk);

#endif
