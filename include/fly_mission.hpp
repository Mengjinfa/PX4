#ifndef FLYE_MISSION_HPP
#define FLYE_MISSION_HPP

#include "mavsdk_members.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <string>

int fly_mission(Mavsdk_members &mavsdk, const std::string &plan_file); // 综合函数：根据文件或自定义创建航点，上传并执行任务

std::string determine_mission_file_path(const std::string &mission_type,
                                        const std::string &base_path = "/home/senen/桌面/receive/",
                                        const std::string &file_ext = ".plan");

#endif // FLIGHT_ROUTE_MISSION_HPP