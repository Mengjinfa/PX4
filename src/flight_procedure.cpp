#include "flight_procedure.hpp"
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>

using namespace mavsdk;

// 起飞和降落操作处理（带状态监测）
// 功能：解锁无人机，起飞到指定高度，并监测起飞状态
// 参数：context - 无人机上下文对象，takeoff_altitude_m - 起飞高度（米），timeout_sec - 超时时间（秒）
// 返回值：成功返回0，失败返回相应错误码
// 错误码：-1 - 系统未就绪，1 - 设置起飞高度失败
//         2 - 解锁失败，3 - 起飞命令发送失败
int arming_and_takeoff(Mavsdk_members &mavsdk, float takeoff_altitude_m, float timeout_sec)
{
    Action &action = mavsdk.action;

    // 设置起飞高度
    const Action::Result set_alt_result = action.set_takeoff_altitude(takeoff_altitude_m);
    if (set_alt_result != Action::Result::Success)
    {
        std::cerr << "设置起飞高度失败: " << set_alt_result << "\n";
        return 1;
    }
    std::cout << "起飞高度设置为: " << takeoff_altitude_m << " 米\n";

    // 解锁
    std::cout << "准备解锁...\n";
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        std::cerr << "解锁失败: " << arm_result << "\n";
        return 2;
    }
    std::cout << "无人机已解锁，电机运行中...\n";

    // 起飞
    std::cout << "开始起飞...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        std::cerr << "起飞命令发送失败: " << takeoff_result << "\n";
        return 3;
    }
}

// 降落函数
int land_and_disarm(Mavsdk_members &mavsdk, float timeout_sec)
{
    Action &action = mavsdk.action;

    std::cout << "\n开始降落...\n";
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        std::cerr << "降落命令发送失败: " << land_result << "\n";
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(10)); // 等待电机停止

    std::cout << "降落完成，正在上锁...\n";
    const Action::Result disarm_result = action.disarm();
    if (disarm_result == Action::Result::Success)
    {
        std::cout << "无人机已安全上锁\n";
        return 2;
    }
    else
    {
        std::cerr << "上锁失败: " << disarm_result << "\n";
        return 3;
    }
}

// 处理Offboard模式下的飞行位置控制
// 功能：在Offboard模式下控制无人机飞行到指定位置
// 参数：north_m - 北向偏移量（米），east_m - 东向偏移量（米），down_m - 向下偏移量（米），yaw_deg - 偏航角（度）
// 返回值：成功返回true，失败返回false
bool offboard_flight_position(Mavsdk_members &mavsdk, float north_m, float east_m, float down_m, float yaw_deg)
{
    Offboard &offboard = mavsdk.offboard;

    // 创建位置指令
    Offboard::PositionNedYaw position_ned = {};
    position_ned.north_m = north_m;
    position_ned.east_m = east_m;
    position_ned.down_m = down_m;
    position_ned.yaw_deg = yaw_deg;

    // 初始化Offboard模式（发送空指令）
    std::cout << "初始化Offboard模式...\n";
    offboard.set_position_ned({});

    try
    {
        // 启动Offboard模式
        std::cout << "启动Offboard模式...\n";
        const Offboard::Result offboard_start_result = offboard.start();
        if (offboard_start_result != Offboard::Result::Success)
        {
            std::cerr << "offboard模式 启动失败: " << offboard_start_result << "\n";
            return false;
        }

        // 发送目标位置指令（持续5秒或直到中断）
        const auto start_time = std::chrono::steady_clock::now();
        std::cout << "发送目标位置指令...\n";
        while (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start_time).count() < 5)
        {
            offboard.set_position_ned(position_ned);                     // 设置目标位置
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // 5Hz更新频率
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "offboard异常: " << e.what() << "\n";
        return false;
    }
}
