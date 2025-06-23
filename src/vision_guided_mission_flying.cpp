#include "vision_guided_mission_flying.hpp"
#include "logger.hpp"
#include "mqtt_client.hpp"
#include "target_tracker.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>
#include <fstream>
#include <functional>
#include <future>
#include <nlohmann/json.hpp>
#include <pugixml.hpp>
#include <thread>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// 创建一个任务项，包含位置、速度、云台角度和相机动作等信息
Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    bool is_fly_through,
    float gimbal_pitch_deg,
    float gimbal_yaw_deg,
    Mission::MissionItem::CameraAction camera_action)
{
    Mission::MissionItem new_item{};
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

// 从KML文件中读取任务点
std::vector<Mission::MissionItem> read_kml_file(const std::string &filename)
{
    std::vector<Mission::MissionItem> items;

    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str()))
    {
        logging::get_logger()->warn("Failed to load KML file: {}", filename);
        return items;
    }

    auto coords_node = doc.select_node("//coordinates");
    if (!coords_node)
    {
        logging::get_logger()->warn("No coordinates found in KML file.");
        return items;
    }

    std::istringstream iss(coords_node.node().value());
    std::string coord;
    while (std::getline(iss, coord, ' '))
    {
        std::istringstream coord_stream(coord);
        float longitude, latitude, altitude;
        char comma;
        coord_stream >> longitude >> comma >> latitude >> comma >> altitude;

        items.push_back(make_mission_item(latitude, longitude, altitude, 5.0f, true, 0.0f, 0.0f, Mission::MissionItem::CameraAction::None));
    }

    return items;
}

// 初始化参数，设置标志位
void initparam()
{
    is_take_off.store(1);
    start_posctl.store(0);
    start_detect.store(0);
}

/**
 * 执行无人机解锁并起飞
 * @param action 动作控制接口，用于发送解锁和起飞命令
 * @return 成功返回true，失败返回false
 *
 * 功能说明：
 * 1. 发送解锁命令并等待成功
 * 2. 发送起飞命令并等待起飞完成
 * 3. 等待检测触发信号（start_detect）
 * 4. 通过MQTT发布状态更新
 */
bool arm_and_takeoff(Action &action)
{
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        logging::get_logger()->error("Arming failed");
        return false;
    }

    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        logging::get_logger()->error("Takeoff failed");
        return false;
    }

    // 等待检测触发信号（由mqtt 消息"1" 设置start_detect为true）
    while (!start_detect)
    {
        sleep_for(seconds(2));
        // mqtt_client::publish(PX4_REPLY, "正在等待进入检测");
    }

    return true;
}

// 开始检测任务
void start_detection(Mavsdk_members &context)
{
    logging::get_logger()->info("start detecting");

    // 启动着陆检测 并 发送控制命令
    detectLandingPadAndSendCommand(context);
}

/**
 * 执行降落 并 解除无人机武装状态
 * @param action 动作控制接口，用于发送降落和解除武装命令
 * @param telemetry 遥测数据接口，用于获取无人机状态
 * @return 成功返回true，失败返回false
 *
 * 功能说明：
 * 1. 发送降落命令并等待降落完成
 * 2. 尝试解除无人机武装状态
 * 3. 持续监控状态直至安全退出
 * 4. 通过MQTT发布状态更新
 */
bool land_and_disarm(Action &action, Telemetry &telemetry)
{
    // 延时确保系统稳定
    sleep_for(std::chrono::seconds(2));

    // 发布降落开始状态
    // mqtt_client::publish(PX4_REPLY, "降落任务");
    logging::get_logger()->info("Landing...");

    // 发送降落命令
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        logging::get_logger()->error("Land failed");
        return false;
    }

    // 等待无人机完成降落（检测是否在空中）
    while (telemetry.in_air())
    {
        logging::get_logger()->info("Vehicle is landing...");
        sleep_for(std::chrono::seconds(1));
    }

    // 降落完成通知
    // mqtt_client::publish(PX4_REPLY, "降落完成");
    logging::get_logger()->info("Landed!");

    // 尝试解除武装
    if (telemetry.armed())
    {
        logging::get_logger()->info("Attempting to disarm...");

        // 发送解除武装命令
        if (action.disarm() != Action::Result::Success)
        {
            // 解除武装失败
            logging::get_logger()->error("Disarm command failed! Check safety switches or throttle position.");
        }

        // 等待系统响应
        sleep_for(std::chrono::seconds(2));

        // 检查解除武装结果
        if (telemetry.armed())
        {
            logging::get_logger()->warn("Warning: Vehicle remains armed after disarm command!");
        }
        else
        {
            logging::get_logger()->info("Vehicle successfully disarmed.");
        }
    }

    // 安全保障：确保无人机已解除武装
    while (telemetry.armed())
    {
        sleep_for(std::chrono::seconds(1));
    }

    // 任务完成通知
    // mqtt_client::publish(PX4_REPLY, "Disarmed, exiting.");
    logging::get_logger()->info("无人机已解除武装");

    return true;
}

// 定义航点结构体
struct Waypoint
{
    bool auto_continue;
    float altitude;
    float speed;
    double latitude;
    double longitude;
    uint16_t command;
};

// 解析.plan文件，提取任务点和速度信息
std::vector<Waypoint> parse_plan_file(const std::string &filename, float &default_cruise_speed)
{
    std::vector<Waypoint> waypoints;
    std::ifstream file(filename);
    if (!file.is_open())
    {
        throw std::runtime_error("无法打开.plan文件");
    }

    nlohmann::json data;
    file >> data;

    // 提取任务级默认速度
    if (data["mission"].contains("cruiseSpeed"))
    {
        default_cruise_speed = data["mission"]["cruiseSpeed"];
    }
    else
    {
        default_cruise_speed = 15.0f; // 默认值
    }

    // 解析任务项
    for (const auto &item : data["mission"]["items"])
    {
        if (item["type"] == "SimpleItem")
        {
            Waypoint wp;
            wp.latitude = item["params"][4];
            wp.longitude = item["params"][5];
            wp.altitude = item["params"][6];
            wp.command = item["command"];
            wp.auto_continue = item["autoContinue"];

            // 优先使用任务项中的速度参数
            if (item["params"].size() > 2 && !item["params"][2].is_null())
            {
                wp.speed = item["params"][2];
            }
            else
            {
                wp.speed = default_cruise_speed;
            }

            waypoints.push_back(wp);
        }
    }

    return waypoints;
}

// 根据航点信息创建任务项
Mission::MissionItem create_mission_item(const Waypoint &wp)
{
    Mission::MissionItem item;
    item.latitude_deg = wp.latitude;
    item.longitude_deg = wp.longitude;
    item.relative_altitude_m = wp.altitude;
    item.speed_m_s = wp.speed;
    item.is_fly_through = wp.auto_continue;
    item.gimbal_pitch_deg = 0.0f;
    item.gimbal_yaw_deg = 0.0f;
    item.camera_action = Mission::MissionItem::CameraAction::None;
    item.loiter_time_s = 0;
    item.camera_photo_interval_s = 0;

    return item;
}

/**
 * 执行无人机飞行任务
 * @param missionId 任务唯一标识符
 * @param context 无人机上下文对象，包含任务、遥测和动作控制
 * @param waypoint_route 指定的航线路径代码（如"AB","AC"等）
 * @return 成功返回0，失败返回非零错误码
 */
int fly_mission(const std::string &missionId, Mavsdk_members &mavsdk, const std::string &waypoint_route)
{
    // 从上下文获取关键组件引用
    MissionRaw &mission_raw = mavsdk.mission_raw;
    Telemetry &telemetry = mavsdk.telemetry;
    // Mission &mission = mavsdk.mission;
    Action &action = mavsdk.action;

    logging::get_logger()->info("System ready");

    // 定义有效的预定义航线路径列表
    std::vector<std::string> valid_routes = {"AB", "AC", "BA", "BC", "CA", "CB"};
    bool is_valid_route = std::find(valid_routes.begin(), valid_routes.end(), waypoint_route) != valid_routes.end();

    // 有效航线：执行航线
    if (is_valid_route)
    {
        // 执行预定义航线任务
        std::string mission_file = "mission_" + waypoint_route + ".plan";
        logging::get_logger()->info("Loading mission from file: {}", mission_file);

        // 导入QGroundControl任务文件
        auto import_result = mission_raw.import_qgroundcontrol_mission(mission_file);
        if (import_result.first != MissionRaw::Result::Success)
        {
            logging::get_logger()->error("航线读取失败");
            return 1;
        }

        // 准备起飞
        logging::get_logger()->info("Arming...");
        const Action::Result arm_result = action.arm();
        if (arm_result != Action::Result::Success)
        {
            logging::get_logger()->error("解锁失败");
            return 1;
        }

        // 上传任务到无人机
        auto upload_result = mission_raw.upload_mission(import_result.second.mission_items);
        if (upload_result != MissionRaw::Result::Success)
        {
            logging::get_logger()->error("航线上传失败");
            return 1;
        }

        // 开始执行任务
        auto start_mission_result = mission_raw.start_mission();
        if (start_mission_result != MissionRaw::Result::Success)
        {
            std::cerr << "任务启动失败: " << start_mission_result << '\n';
            return 1;
        }

        // 设置任务进度回调
        auto prom = std::promise<void>();
        auto fut = prom.get_future();

        MissionRaw::MissionProgressHandle handle = mission_raw.subscribe_mission_progress([&](MissionRaw::MissionProgress progress)
                                                                                          {
            // 打印任务进度
            std::cout << "Progress: " << progress.current << "/" << progress.total << std::endl;
            
            // 任务完成时通知主线程
            if (progress.current == progress.total) 
            {
                mission_raw.unsubscribe_mission_progress(handle);
                prom.set_value();
            } });

        // 等待任务完成，超时时间60秒
        if (fut.wait_for(std::chrono::seconds(60)) != std::future_status::ready)
        {
            logging::get_logger()->error("未在60秒内完成航线");
        }
        fut.get(); // 获取结果，确保任务完成

        // 等待检测触发信号
        while (!start_detect)
        {
            sleep_for(std::chrono::seconds(2));
            logging::get_logger()->info("正在等待进入检测");
            // mqtt_client::publish(PX4_REPLY, "正在等待进入检测");
        }

        // 执行检测任务
        start_detection(mavsdk);

        // 降落并解除锁定
        if (!land_and_disarm(action, telemetry))
        {
            return 1;
        }
    }
    // 无效航线：执行通用任务
    else
    {
        // 执行通用任务（无预定义航线）
        if (!arm_and_takeoff(action))
        {
            return 1;
        }

        // 等待检测触发信号
        while (!start_detect)
        {
            sleep_for(std::chrono::seconds(2));
            logging::get_logger()->info("正在等待进入检测");
            // mqtt_client::publish(PX4_REPLY, "正在等待进入检测");
        }

        // 执行检测任务
        start_detection(mavsdk);

        // 降落并解除锁定
        if (!land_and_disarm(action, telemetry))
        {
            return 1;
        }
    }

    // 重置系统参数（任务结束后）
    initparam();

    return 0;
}
