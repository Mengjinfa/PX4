#include "takeoff_and_land.hpp"
#include <cstdint>
// #include <mavsdk/mavsdk.h>
// #include <mavsdk/plugins/action/action.h>
// #include <mavsdk/plugins/telemetry/telemetry.h>
#include <iostream>
#include <future>
#include <memory>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;
// int takeoff_and_land(int enable,Action& action,Telemetry& telemetry)
// {
//     std::cerr << "8888888888888888888888888" << std::endl;
//     // Telemetry{system.value()};
//     // auto telemetry = Telemetry{mavlink_manager.get_system()};
//     // auto action = Action{mavlink_manager.get_system()};
//     // auto telemetry = mavlink_manager.get_telemetry();
//     // auto action = mavlink_manager.get_action();
//     // We want to listen to the altitude of the drone at 1 Hz.
//     const auto set_rate_result = telemetry.set_rate_position(1.0);
//     if (set_rate_result != Telemetry::Result::Success) {
//         std::cerr << "Setting rate failed: " << set_rate_result << '\n';
//         return 1;
//     }

//     // Set up callback to monitor altitude while the vehicle is in flight
//     telemetry.subscribe_position([](Telemetry::Position position) {
//         std::cout << "Altitude: " << position.relative_altitude_m << " m\n";
//     });

//     // Arm vehicle
//     std::cout << "Arming...\n";
//     const Action::Result arm_result = action.arm();

//     if (arm_result != Action::Result::Success) {
//         std::cerr << "Arming failed: " << arm_result << '\n';
//         return 1;
//     }

//     // Take off
//     std::cout << "Taking off...\n";
//     const Action::Result takeoff_result = action.takeoff();
//     if (takeoff_result != Action::Result::Success) {
//         std::cerr << "Takeoff failed: " << takeoff_result << '\n';
//         return 1;
//     }

//     // Let it hover for a bit before landing again.
//     sleep_for(seconds(10));

//     std::cout << "Landing...\n";
//     const Action::Result land_result = action.land();
//     if (land_result != Action::Result::Success) {
//         std::cerr << "Land failed: " << land_result << '\n';
//         return 1;
//     }

//     // Check if vehicle is still in air
//     while (telemetry.in_air()) {
//         std::cout << "Vehicle is landing...\n";
//         sleep_for(seconds(1));
//     }
//     std::cout << "Landed!\n";

//     // We are relying on auto-disarming but let's keep watching the telemetry for a bit longer.
//     sleep_for(seconds(3));
//     std::cout << "Finished...\n";

//     return 0;
// }
const Offboard::PositionNedYaw position_ned{
    .north_m = 0,
    .east_m = 0,
    .down_m = -1.5,  // 注意正值表示向下
    .yaw_deg = 0
};

static std::atomic<int> current_enable(1);

int takeoff_and_land(int enable, Action& action, Telemetry& telemetry, Offboard& offboard) {
 
    // 使用原子变量保证线程安全
    current_enable.store(enable);
    if (current_enable == 0) { // 起飞并维持5米高度
        // 设置高度订阅
        const auto set_rate_result = telemetry.set_rate_position(1.0);
        if (set_rate_result != Telemetry::Result::Success) {
            std::cerr << "Setting rate failed: " << set_rate_result << '\n';
            return 1;
        }
 
        // 订阅高度数据
        telemetry.subscribe_position([](Telemetry::Position position) {
            std::cout << "Current Altitude: " << position.relative_altitude_m << " m\n";
        });
 
        // 解锁
        std::cout << "Arming...\n";
        const Action::Result arm_result = action.arm();
        if (arm_result != Action::Result::Success) {
            std::cerr << "Arming failed: " << arm_result << '\n';
            return 1;
        }
 
        // 起飞
        std::cout << "Taking off...\n";
        const Action::Result takeoff_result = action.takeoff();
        if (takeoff_result != Action::Result::Success) {
            std::cerr << "Takeoff failed: " << takeoff_result << '\n';
            return 1;
        }
 
        // 进入Offboard模式
        std::cout << "Entering Offboard mode...\n";
        const Offboard::Result offboard_start_result = offboard.start();
        if (offboard_start_result != Offboard::Result::Success) {
            std::cerr << "Failed to enter Offboard mode: " << offboard_start_result << '\n';
            return 1;
        }
 
        // 设置目标高度（NED坐标系，z向下为正）
        // auto target_position = PositionNedYaw{0, 0, -5.0, 0};
        
        // 维持高度循环（直到收到enable=1）
        std::cout << "Maintaining 5m altitude...\n";
        while (current_enable == 0) {  // 持续检查enable状态
            offboard.set_position_ned(position_ned);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 缩短检查间隔
 
            // // 动态高度调整逻辑
            // auto pos = telemetry.position();
            // if (pos.relative_altitude_m > 5.5) {
            //     position_ned.down_m = -5.0;  // 保持目标高度
            // } else if (pos.relative_altitude_m < 4.5) {
            //     position_ned.down_m = -5.0;  // 防止负高度
            // }
        }
 
        // 停止Offboard模式
        offboard.stop();
 
        // 等待降落完成
        while (telemetry.in_air()) {
            std::cout << "Vehicle is landing...\n";
            std::this_thread::sleep_for(seconds(1));
        }
 
    } else if (current_enable == 1) { // 降落
        // 检查是否在飞行中
        if (!telemetry.in_air()) {
            std::cerr << "Vehicle is already on ground." << std::endl;
            return 0;
        }
 
        // 降落
        std::cout << "Landing...\n";
        const Action::Result land_result = action.land();
        if (land_result != Action::Result::Success) {
            std::cerr << "Land failed: " << land_result << '\n';
            return 1;
        }
 
        // 等待降落完成
        while (telemetry.in_air()) {
            std::cout << "Vehicle is landing...\n";
            std::this_thread::sleep_for(seconds(1));
        }
        std::cout << "Landed!\n";
 
    } else {
        std::cerr << "Invalid enable value: " << current_enable << std::endl;
        return 1;
    }
 
    std::cout << "Operation completed.\n";
    return 0;
}