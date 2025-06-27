#ifndef USER_TASK_HPP
#define USER_TASK_HPP

#include "mavsdk_members.hpp"

#include <chrono>
#include <cmath>
#include <string>

using namespace mavsdk;

void handleTestMessage(const std::vector<unsigned char> &payload);
void user_task(Mavsdk_members &mavsdk);

#endif
