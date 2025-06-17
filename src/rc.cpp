#include "rc.hpp"
#include "target_tracker.hpp"
#include <iostream>
#include <chrono>
#include <thread>
// #include <mavsdk/plugins/offboard/offboard.h>
// #include <mavsdk/plugins/action/action.h>
// #include <mavsdk/plugins/telemetry/telemetry.h>



using namespace mavsdk;


// void set_offboard_mode(MavlinkPassthrough& mavlink_passthrough) {
//     mavlink_command_long_t cmd{};
//     cmd.target_system = 1;
//     cmd.target_component = 1;
//     cmd.command = MAV_CMD_DO_SET_MODE;
//     cmd.confirmation = 0;
//     cmd.param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
//     cmd.param2 = 0;
//     cmd.param3 = 0;
//     cmd.param4 = 0;
//     cmd.param5 = 0;
//     cmd.param6 = 0;
//     cmd.param7 = 0;

//     mavlink_message_t message;
//     mavlink_msg_command_long_encode(0, 0, &message, &cmd);
//     mavlink_passthrough.send_message(message);
//     std::cout << "Sent command to set OFFBOARD mode." << std::endl;
// }




// void setup_offboard_mode(Action& action, Offboard& offboard) {
//     // 切换到 Offboard 模式
//     Action::Result result = action.set_flight_mode(Action::FlightMode::Offboard);
//     if (result != Action::Result::Success) {
//         std::cerr << "Failed to set flight mode to Offboard: " << action.result_str(result) << std::endl;
//         return;
//     }

//     // 解锁无人机
//     result = action.arm();
//     if (result != Action::Result::Success) {
//         std::cerr << "Failed to arm the drone: " << action.result_str(result) << std::endl;
//         return;
//     }

//     std::cout << "Drone armed and in Offboard mode." << std::endl;
// }



void monitor_telemetry(Telemetry& telemetry) {
    telemetry.subscribe_battery([](Telemetry::Battery battery) {
        std::cout << "Battery: " << battery.remaining_percent * 100.0f << "%, Voltage: " << battery.voltage_v << "V" << std::endl;
    });

    telemetry.subscribe_in_air([&telemetry](bool in_air) {
        if (in_air) {
            std::cout << "Drone is airborne." << std::endl;
        } else {
            std::cout << "Drone is on ground." << std::endl;
        }
    });

    telemetry.subscribe_position([](Telemetry::Position position) {
        std::cout << "Position: Lat " << position.latitude_deg << ", Lon " << position.longitude_deg
                  << ", Alt " << position.absolute_altitude_m << "m" << std::endl;
    });
}


int rc_control(MavlinkPassthrough& mavlink_passthrough,Action& action ,Offboard& offboard,Telemetry& telemetry) {
    // //使用mavlink_passthrough解析mavlink_message_t的值

    //   // 创建回调函数用于处理接收到的 MAVLink 消息
    // auto message_callback = [&offboard](const mavlink_message_t& msg) {
    //     // 解析 RC 通道数据
    //     mavlink_rc_channels_t rc_channels;
    //     mavlink_msg_rc_channels_decode(&msg, &rc_channels);

    //     std::cout << "Received RC channel data." << std::endl;

    //     // 检查是否有至少 8 个通道
    //     if (rc_channels.chancount >= 8) {
    //         uint16_t channel8_value = rc_channels.chan7_raw; // 索引从0开始，第8通道为索引7
    //         std::cout << "Channel 8 value: " << channel8_value << std::endl;
    //         if(channel8_value >= 900 && channel8_value <= 1100)
    //         {
    //            //设置飞机模式为OFFBOARD模式
    //             // set_offboard_mode(action,offboard);

    //             Offboard::VelocityBodyYawspeed zero_velocity{};
    //             offboard.set_velocity_body(zero_velocity); // 初始速度为零
    //             offboard.start();

    //              // 启动视觉检测与控制线程
    //             std::thread detection_thread([&offboard]() {
    //                 detectLandingPadAndSendCommand(offboard);
    //             });

    //             // 等待用户输入退出
    //             std::cout << "Press Enter to stop Offboard mode and exit..." << std::endl;
    //             std::cin.get();



    //               // 停止 Offboard 模式并锁定无人机
    //             offboard.stop();
    //             //action.disarm();
    //             std::cout << "Offboard mode stopped and drone disarmed." << std::endl;

                
                
    //         }
    //     }
    // };

    // // 订阅 RC_CHANNELS 类型的消息
    // auto handle = mavlink_passthrough.subscribe_message(MAVLINK_MSG_ID_RC_CHANNELS, message_callback);

    // // 保持程序运行，直到用户中断
    // try {
    //     while (true) {
    //         std::this_thread::sleep_for(std::chrono::seconds(1));
    //     }
    // } catch (const std::exception& e) {
    //     std::cerr << "Interrupted: " << e.what() << std::endl;
    // }

    return 0; // 正常退出
}

















