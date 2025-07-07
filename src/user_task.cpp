#include "user_task.hpp"

#include "fly_mission.hpp"
#include "landing_state_machine.hpp"

#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <unistd.h>

UserTask user_task;

/**
 * @brief 处理接收到的测试主题消息
 *
 * 该函数作为MQTT消息回调函数，负责解析接收到的JSON格式消息，
 * 根据不同的命令类型设置相应的标志位，并存储相关数据供主函数使用。
 *
 * @param payload 接收到的消息负载数据，以无符号字符向量形式存储
 */
void handleTestMessage(const std::vector<unsigned char> &payload)
{
    try
    {
        std::string payloadStr(payload.begin(), payload.end()); // 将二进制负载数据转换为字符串
        json msg = json::parse(payloadStr);                     // 使用JSON解析库解析消息字符串
        std::string command = msg.value("command", "");         // 从JSON对象中提取"command"字段，默认值为空字符串

        // 根据不同的命令类型执行相应操作
        if (command == "takeoff")
        {
            user_task.takeoff_task_flag = true;
            std::cout << "收到起飞命令" << std::endl;
            mqtt_client::Instance()->sendMessage(REPLAY_TOPIC, "收到起飞命令");
        }
        else if (command == "landing")
        {
            user_task.landing_task_flag = true;
            std::cout << "收到视觉降落命令" << std::endl;
            mqtt_client::Instance()->sendMessage(REPLAY_TOPIC, "收到视觉降落命令");
        }
        else if (command == "waypoint")
        {
            user_task.waypoint_task_flag = true;
            std::cout << "收到航点任务命令" << std::endl;
            mqtt_client::Instance()->sendMessage(REPLAY_TOPIC, "收到航点任务命令");
        }
        else if (command == "land")
        {
            user_task.land_mode_flag = true;
            std::cout << "收到降落模式任务命令" << std::endl;
            mqtt_client::Instance()->sendMessage(REPLAY_TOPIC, "收到降落模式任务命令");
        }
        else
        {
            std::cout << "收到命令错误" << std::endl;
            mqtt_client::Instance()->sendMessage(REPLAY_TOPIC, "收到命令错误");
        }
    }
    catch (const json::parse_error &e)
    {
        // 捕获JSON解析过程中可能出现的异常
        std::cerr << "解析消息失败: " << e.what() << std::endl;
        std::cerr << "错误位置: " << e.byte << " 字节" << std::endl;
    }
}

// 用户任务过程
void userTaskProcedure(Mavsdk_members &mavsdk)
{
    if (user_task.takeoff_task_flag)
    {
        user_task.takeoff_task_flag = false;
        std::cout << "执行起飞任务" << std::endl;
        arming_and_takeoff(mavsdk, 5.0);
    }

    if (user_task.land_mode_flag)
    {
        user_task.land_mode_flag = false;
        std::cout << "执行降落模式任务" << std::endl;
        land_and_disarm(mavsdk);
    }

    if (user_task.landing_task_flag)
    {
        std::cout << "执行识别降落任务" << std::endl;
        user_task.landing_task_flag = landing_state_machine::Instance()->StartStateMachine();
    }

    if (user_task.waypoint_task_flag)
    {
        user_task.waypoint_task_flag = false;
        std::cout << "执行航点任务" << std::endl;
        fly_mission(mavsdk, "");
    }
}