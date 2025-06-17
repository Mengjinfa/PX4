#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <pugixml.hpp>

#include <chrono>
#include <functional>
#include <future>
#include <thread>

#include "vision_guided_mission_flying.hpp"
#include "target_tracker.hpp"

// 引入全局日志模块
#include "logger.hpp"

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

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

std::vector<Mission::MissionItem> read_kml_file(const std::string& filename) {
    std::vector<Mission::MissionItem> items;

    pugi::xml_document doc;
    if (!doc.load_file(filename.c_str())) {
        logging::get_logger()->warn("Failed to load KML file: {}", filename);
        return items;
    }

    auto coords_node = doc.select_node("//coordinates");
    if (!coords_node) {
        logging::get_logger()->warn("No coordinates found in KML file.");
        return items;
    }

    std::istringstream iss(coords_node.node().value());
    std::string coord;
    while (std::getline(iss, coord, ' ')) {
        std::istringstream coord_stream(coord);
        float longitude, latitude, altitude;
        char comma;
        coord_stream >> longitude >> comma >> latitude >> comma >> altitude;

        items.push_back(make_mission_item(
            latitude, longitude, altitude,
            5.0f, true, 0.0f, 0.0f,
            Mission::MissionItem::CameraAction::None));
    }

    return items;
}

std::atomic<int> start_detect(0);
std::atomic<int> find_landmark(0);
std::atomic<int> start_posctl(0);
std::atomic<int> is_take_off(1);

void initparam() {
    is_take_off.store(1);
    start_posctl.store(0);
    start_detect.store(0);
}

int fly_mission(
    const std::string& missionId,
    Action& action,
    Mission& mission,
    Telemetry& telemetry,
    Offboard& offboard,
    Camera& camera,
    MavlinkPassthrough& mavlink_passthrough)
{
    logging::get_logger()->info("System ready");
    logging::get_logger()->info("Creating and uploading mission");

    logging::get_logger()->info("Arming...");
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        logging::get_logger()->error("Arming failed:");
        return 1;
    }

    logging::get_logger()->info("Taking off...");
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        logging::get_logger()->error("Takeoff failed");
        return 1;
    }

    logging::get_logger()->info("正在起飞");
    while (!start_detect) {
        sleep_for(seconds(1));
        logging::get_logger()->info("正在等待进入检测");
    }

    logging::get_logger()->info("start detecting112233...");
    detectLandingPadAndSendCommand(offboard, telemetry, mavlink_passthrough);

    auto current_mode = telemetry.flight_mode();
    // logging::get_logger()->info("Current flight mode: {}", current_mode);

    sleep_for(seconds(2));

    logging::get_logger()->info("Landing...");
    const Action::Result land_result = action.land();
    if (land_result != Action::Result::Success) {
        logging::get_logger()->error("Land failed");
        return 1;
    }

    while (telemetry.in_air()) {
        logging::get_logger()->info("Vehicle is landing...");
        sleep_for(seconds(1));
    }
    logging::get_logger()->info("Landed!");

    if (telemetry.armed()) {
        logging::get_logger()->info("Attempting to disarm...");

        if (action.disarm() != Action::Result::Success) {
            logging::get_logger()->error("Disarm command failed! Check safety switches or throttle position.");
        }

        sleep_for(std::chrono::seconds(2));

        if (telemetry.armed()) {
            logging::get_logger()->warn("Warning: Vehicle remains armed after disarm command!");
        } else {
            logging::get_logger()->info("Vehicle successfully disarmed.");
        }
    }

    while (telemetry.armed()) {
        sleep_for(seconds(1));
    }
    logging::get_logger()->info("Disarmed, exiting.");

    initparam();

    return 0;
}