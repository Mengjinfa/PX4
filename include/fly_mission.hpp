#ifndef FLYE_MISSION_HPP
#define FLYE_MISSION_HPP

#include "mavsdk_members.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <string>
#include <vector>

int fly_mission(Mavsdk_members &mavsdk, const std::string &plan_file); // 综合函数：根据文件或自定义创建航点，上传并执行任务

#endif // FLIGHT_ROUTE_MISSION_HPP