#include "takeoff_and_land.hpp"
#include "target_tracker.hpp"
#include "telemetry_monitor.hpp"
#include <cmath> // 用于计算距离
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;
static std::atomic<int> current_enable(1);

// 在 NED（北东下）坐标系中输入位置指令，并输入偏航角
const Offboard::PositionNedYaw position_NED = {
    1.0f,  // north_m
    1.0f,  // east_m
    -5.0f, // down_m (注意正值表示向下)
    90.0f  // yaw_deg (90度偏航角，正东方向)
};

// 处理起飞和降落操作
int takeoff_and_land(int enable, Mavsdk_members &mavsdk)
{
    // 获取无人机控制相关组件
    Telemetry &telemetry = mavsdk.telemetry;
    Offboard &offboard = mavsdk.offboard;
    Action &action = mavsdk.action;

    // 使用原子变量保证线程安全
    current_enable.store(enable);

    // 起飞 + 悬停
    if (current_enable == 0)
    {
        // 设置位置数据更新频率：控制无人机发送位置信息（如经纬度、高度）的频率，单位为赫兹 (Hz)。
        const auto set_rate_result = telemetry.set_rate_position(5.0);
        if (set_rate_result != Telemetry::Result::Success)
        {
            std::cerr << "设置位置数据更新频率失败: " << set_rate_result << '\n';
            return 1;
        }

        // 解锁
        std::cout << "解锁...\n";
        const Action::Result arm_result = action.arm();
        if (arm_result != Action::Result::Success)
        {
            std::cerr << "解锁 失败: " << arm_result << '\n';
            return 2;
        }

        // 设置起飞高度为10米（相对于起飞位置）
        const float takeoff_altitude_m = 10.0f;
        action.set_takeoff_altitude(takeoff_altitude_m);
        std::cout << "设置起飞高度为: " << takeoff_altitude_m << " 米\n";

        // 起飞
        std::cout << "起飞...\n";
        const Action::Result takeoff_result = action.takeoff();
        if (takeoff_result != Action::Result::Success)
        {
            std::cerr << "起飞失败: " << takeoff_result << '\n';
            return 3;
        }

        // 等待起飞完成
        sleep_for(seconds(15));

        // 发布目标位置
        offboard.set_position_ned(position_NED);

        // 进入Offboard模式
        std::cout << "进入 offboard 模式...\n";
        const Offboard::Result offboard_start_result = offboard.start();
        if (offboard_start_result != Offboard::Result::Success)
        {
            std::cerr << "进入 offboard 模式 失败: " << offboard_start_result << '\n';
            return 4;
        }

        // 发布目标位置
        offboard.set_position_ned(position_NED);

        // 启动目标检测线程
        detectLandingPadAndSendCommand(mavsdk);
    }
    // 降落
    else if (current_enable == 1)
    {
        // 检查是否在飞行中
        if (!telemetry.in_air())
        {
            std::cerr << "无人机已经停在地面了" << std::endl;
            return 0;
        }

        // 降落
        std::cout << "降落...\n";
        const Action::Result land_result = action.land();
        if (land_result != Action::Result::Success)
        {
            std::cerr << "降落 失败: " << land_result << '\n';
            return 1;
        }

        // 等待降落完成
        while (telemetry.in_air())
        {
            std::cout << "无人机正在降落...\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        std::cout << "Landed!\n";
    }
    else
    {
        std::cerr << "无效的启用值: " << current_enable << std::endl;
        return 1;
    }

    std::cout << "操作已完成\n";
    return 0;
}