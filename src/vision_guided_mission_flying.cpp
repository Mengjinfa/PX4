#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <pugixml.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include "target_tracker.hpp"
#include "vision_guided_mission_flying.hpp"

// 引入全局日志模块
#include "logger.hpp"

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

Mission::MissionItem make_mission_item(
    double latitude_deg,                              // 纬度（度）
    double longitude_deg,                             // 经度（度）
    float relative_altitude_m,                        // 相对高度（米）
    float speed_m_s,                                  // 飞行速度（米/秒）
    bool is_fly_through,                              // 是否飞越点（非悬停）
    float gimbal_pitch_deg,                           // 俯仰角（度）
    float gimbal_yaw_deg,                             // 偏航角（度）
    Mission::MissionItem::CameraAction camera_action) // 相机动作
{
    Mission::MissionItem new_item{};

    // 填充任务项参数
    new_item.latitude_deg = latitude_deg;
    new_item.longitude_deg = longitude_deg;
    new_item.relative_altitude_m = relative_altitude_m;
    new_item.speed_m_s = speed_m_s;
    new_item.is_fly_through = is_fly_through;
    new_item.gimbal_pitch_deg = gimbal_pitch_deg;
    new_item.gimbal_yaw_deg = gimbal_yaw_deg;
    new_item.camera_action = camera_action;
    return new_item;
}

std::vector<Mission::MissionItem> read_kml_file(const std::string &filename)
{
    std::vector<Mission::MissionItem> items;

    pugi::xml_document doc;
    // 加载KML文件
    if (!doc.load_file(filename.c_str()))
    {
        logging::get_logger()->warn("Failed to load KML file: {}", filename);
        return items;
    }

    // 定位坐标节点
    auto coords_node = doc.select_node("//coordinates");
    if (!coords_node)
    {
        logging::get_logger()->warn("No coordinates found in KML file.");
        return items;
    }

    // 解析坐标数据（格式：经度,纬度,高度）
    std::istringstream iss(coords_node.node().value());
    std::string coord;
    while (std::getline(iss, coord, ' '))
    {
        std::istringstream coord_stream(coord);
        float longitude, latitude, altitude;
        char comma;
        coord_stream >> longitude >> comma >> latitude >> comma >> altitude;

        // 将KML坐标转换为任务项
        items.push_back(make_mission_item(
            latitude, longitude, altitude,
            5.0f, true, 0.0f, 0.0f,
            Mission::MissionItem::CameraAction::None));
    }

    return items;
}

std::atomic<int> start_detect(0);  // 开始视觉检测标志
std::atomic<int> find_landmark(0); // 找到地标标志
std::atomic<int> start_posctl(0);  // 开始位置控制标志
std::atomic<int> is_take_off(1);   // 起飞状态标志

void initparam()
{
    is_take_off.store(1);  // 重置起飞状态为"已起飞"
    start_posctl.store(0); // 关闭位置控制
    start_detect.store(0); // 关闭视觉检测
}

int fly_mission(
    const std::string &missionId,            // 任务ID（未使用）
    Action &action,                          // 动作控制插件
    Mission &mission,                        // 任务规划插件
    Telemetry &telemetry,                    // 遥测数据插件
    Offboard &offboard,                      // 自主飞行模式插件
    Camera &camera,                          // 相机控制插件
    MavlinkPassthrough &mavlink_passthrough) // MAVLink透传插件
{
    logging::get_logger()->info("System ready");
    logging::get_logger()->info("Creating and uploading mission");

    // 解锁
    logging::get_logger()->info("Arming...");
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        logging::get_logger()->error("Arming failed:");
        return 1;
    }

    // 起飞
    logging::get_logger()->info("Taking off...");
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        logging::get_logger()->error("Takeoff failed");
        return 1;
    }
    logging::get_logger()->info("正在起飞");

    // 等待视觉检测开始信号
    while (!start_detect)
    {
        sleep_for(seconds(1));
        logging::get_logger()->info("正在等待进入检测");
    }

    logging::get_logger()->info("start detecting112233...");

    // 执行视觉引导降落垫检测及控制
    detectLandingPadAndSendCommand(offboard, telemetry, mavlink_passthrough);

    // 等待模式切换（注释中未使用，可能用于后续扩展）
    auto current_mode = telemetry.flight_mode();
    // logging::get_logger()->info("Current flight mode: {}", current_mode);

    sleep_for(seconds(2));

    // 降落
    logging::get_logger()->info("Landing...");
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        logging::get_logger()->error("Land failed");
        return 1;
    }

    // 等待无人机落地
    while (telemetry.in_air())
    {
        logging::get_logger()->info("Vehicle is landing...");
        sleep_for(seconds(1));
    }
    logging::get_logger()->info("Landed!");

    // 解除武装
    if (telemetry.armed())
    {
        logging::get_logger()->info("Attempting to disarm...");

        if (action.disarm() != Action::Result::Success)
        {
            logging::get_logger()->error("Disarm command failed! Check safety switches or throttle position.");
        }

        sleep_for(std::chrono::seconds(2));

        if (telemetry.armed())
        {
            logging::get_logger()->warn("Warning: Vehicle remains armed after disarm command!");
        }
        else
        {
            logging::get_logger()->info("Vehicle successfully disarmed.");
        }
    }

    // 等待解除武装完成
    while (telemetry.armed())
    {
        sleep_for(seconds(1));
    }
    logging::get_logger()->info("Disarmed, exiting.");

    // 重置任务参数
    initparam();

    return 0;
}