#include "user_task.hpp"

#include "flight_procedure.hpp"
#include "fly_mission.hpp"
#include "landing_state_machine.hpp"
#include "mqtt_client.hpp"

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <unistd.h>

using namespace mavsdk;

// 全局状态
std::atomic<bool> startMachineFlag(false);

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
        if (command == "start")
        {
            startMachineFlag = true;                  // 设置启动标志为true，通知主函数启动状态机
            std::cout << "收到开始命令" << std::endl; // 处理停止命令的逻辑
        }
        else if (command == "stop")
        {
            std::cout << "收到停止命令" << std::endl; // 处理停止命令的逻辑
        }
    }
    catch (const json::parse_error &e)
    {
        // 捕获JSON解析过程中可能出现的异常
        std::cerr << "解析消息失败: " << e.what() << std::endl;
        std::cerr << "错误位置: " << e.byte << " 字节" << std::endl;
    }
}

void user_task(Mavsdk_members &mavsdk)
{
    static int task_step = 0;
    int fly_mission_flag = 0;

    if (startMachineFlag)
    {
        switch (task_step)
        {
            case 0:
            {
                // 上传自定义任务
                fly_mission_flag = fly_mission(mavsdk, "/home/senen/桌面/hx.plan");
                task_step = 1;
            }
            break;
            case 1:
            {
                if (fly_mission_flag)
                {
                    set_landing_state_flag();
                }
            }
            break;
            case 2:
            {
            }
            break;
            case 3:
            {
            }
            break;
            default: // 异常状态处理
                break;
        }
    }
}

// 获取当前时间点
std::chrono::time_point<std::chrono::system_clock> getCurrentTime()
{
    return std::chrono::system_clock::now();
}