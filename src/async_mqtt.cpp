#include "logger.hpp" // 引入全局日志模块

#include "BeiDouModule.h"
#include "async_mqtt.hpp"
#include "rc.hpp"
#include "takeoff_and_land.hpp"
#include "target_tracker.hpp"
#include "vision_guided_mission_flying.hpp"

#include "sim_camera_module.h"
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <nlohmann/json.hpp>

#include <fcntl.h> // for open
#include <fstream>
#include <unistd.h> // for read

using namespace mavsdk;

std::string subscribePtr = "/gazebo/default/iris/base_link/camera/image";

Mavsdk g_mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}};
std::optional<Action> g_action;
std::optional<Mission> g_mission;
std::optional<Telemetry> g_telemetry;
std::optional<MavlinkPassthrough> g_mavlinkpassthrough;
std::optional<Offboard> g_offboard;
std::optional<Camera> g_camera;

action_listener::action_listener(const std::string &name) : name_(name) {}

void action_listener::on_failure(const mqtt::token &tok)
{
    logging::get_logger()->error("{} failure", name_);
    if (tok.get_message_id() != 0)
        logging::get_logger()->error(" for token: [{}]", tok.get_message_id());
}

void action_listener::on_success(const mqtt::token &tok)
{
    logging::get_logger()->info("{} success", name_);
    if (tok.get_message_id() != 0)
        logging::get_logger()->info(" for token: [{}]", tok.get_message_id());
    auto top = tok.get_topics();
    if (top && !top->empty())
        logging::get_logger()->info("\ttoken topic: '{}', ...", (*top)[0]);
}

int process_start_mission(const std::string &missionId)
{
    if (missionId == "1")
    {
        start_detect.store(start_detect.load() ? 0 : 1);
        logging::get_logger()->info("start_detect: {}", start_detect.load());
        return 0;
    }
    else if (missionId == "21")
    {
        find_landmark.store(find_landmark.load() ? 0 : 1);
        logging::get_logger()->info("find_landmark: {}", find_landmark.load());
        return 0;
    }
    else if (missionId == "2")
    {
        start_posctl.store(start_posctl.load() ? 0 : 1);
        logging::get_logger()->info("start_posctl: {}", start_posctl.load());
        return 0;
    }
    else if (missionId == "3")
    {
        is_take_off.store(is_take_off.load() ? 0 : 1);
        logging::get_logger()->info("is_take_off: {}", is_take_off.load());
        return 0;
    }
    else if (missionId == "4")
    {
        detection_running.store(detection_running.load() ? false : true);
        logging::get_logger()->info("detection_running: {}", detection_running.load());
        return 0;
    }

    int reply_code = fly_mission(missionId, g_action.value(), g_mission.value(), g_telemetry.value(), g_offboard.value(), g_camera.value(), g_mavlinkpassthrough.value());
    return reply_code;
}

void handle_start_mission(const std::string &json_str)
{
    try
    {
        auto root = nlohmann::json::parse(json_str);

        if (root.contains("data") && root["data"].is_object())
        {
            auto data = root["data"];

            if (data.contains("missionId") && data["missionId"].is_string())
            {
                std::string missionId = data["missionId"];
                int result = process_start_mission(missionId);
            }
        }
    }
    catch (const nlohmann::json::parse_error &e)
    {
        logging::get_logger()->error("JSON parse error: {}", e.what());
    }
}

int process_takeoff_land(int enable)
{
    logging::get_logger()->info("777777777777777");
    int reply_code = takeoff_and_land(enable, g_action.value(), g_telemetry.value(), g_offboard.value());
    return reply_code;
}

void handle_takeoff_land(const std::string &json_str)
{
    try
    {
        logging::get_logger()->info("qqqqqq11111111222222222222");
        auto root = nlohmann::json::parse(json_str);

        if (root.contains("data") && root["data"].is_object())
        {
            auto data = root["data"];

            if (data.contains("enable") && data["enable"].is_number_integer())
            {
                int enable = data["enable"];
                int result = process_takeoff_land(enable);
            }
        }
    }
    catch (const nlohmann::json::parse_error &e)
    {
        logging::get_logger()->error("JSON parse error: {}", e.what());
    }
}

int process_enable_auto_landing(int enable)
{

    logging::get_logger()->info("进入高度测量");
    const auto set_rate_result = g_telemetry.value().set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success)
    {
        logging::get_logger()->info("设置高度测量频率失败zzz111");
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }
    //  logging::get_logger()->info("设置高度测量频率失败");
    auto position_handle = g_telemetry.value().subscribe_position([](Telemetry::Position position)
                                                                  { logging::get_logger()->info("当前高度zxkkkk111111：{}", position.relative_altitude_m); });
    return 0;
}

void handle_enable_auto_landing(const std::string &json_str)
{
    try
    {
        auto root = nlohmann::json::parse(json_str);

        if (root.contains("data") && root["data"].is_object())
        {
            auto data = root["data"];

            if (data.contains("enable") && data["enable"].is_number_integer())
            {
                int enable = data["enable"];
                int result = process_enable_auto_landing(enable);
            }
        }
    }
    catch (const nlohmann::json::parse_error &e)
    {
        logging::get_logger()->error("JSON parse error: {}", e.what());
    }
}

int process_start_detect(int enable)
{
    return 0;
}

void handle_start_detect(const std::string &json_str)
{
    try
    {
        auto root = nlohmann::json::parse(json_str);

        if (root.contains("data") && root["data"].is_object())
        {
            auto data = root["data"];

            if (data.contains("enable") && data["enable"].is_number_integer())
            {
                int enable = data["enable"];
                int result = process_start_detect(enable);
            }
        }
    }
    catch (const nlohmann::json::parse_error &e)
    {
        logging::get_logger()->error("JSON parse error: {}", e.what());
    }
}

void handle_upload_mission(const std::string &json_str) {}

void process_command(const std::string &json)
{
    try
    {
        auto j = nlohmann::json::parse(json);

        if (j.contains("scopeId") && j["scopeId"].is_string())
        {
            logging::get_logger()->info("scopeId: {}", j["scopeId"].get<std::string>());
        }

        if (j.contains("method") && j["method"].is_string())
        {
            std::string method = j["method"];
            if (method == "upload_mission")
            {
                handle_upload_mission(json);
            }
            else if (method == "start_mission")
            {
                handle_start_mission(json);
            }
            else if (method == "takeoff_land")
            {
                handle_takeoff_land(json);
            }
            else if (method == "enable_auto_landing")
            {
                handle_enable_auto_landing(json);
            }
            else if (method == "start_detect")
            {
                handle_start_detect(json);
            }
            else
            {
                logging::get_logger()->warn("Unknown method: {}", method);
            }
        }
    }
    catch (const nlohmann::json::parse_error &e)
    {
        logging::get_logger()->error("JSON parse error: {}", e.what());
    }
}

void parse_json_message(const std::string &json_string)
{
    process_command(json_string);
}

callback::callback(mqtt::async_client &cli, mqtt::connect_options &connOpts)
    : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}

void callback::reconnect()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));
    try
    {
        cli_.connect(connOpts_, nullptr, *this);
    }
    catch (const mqtt::exception &exc)
    {
        logging::get_logger()->error("Error: {}", exc.what());
        if (++nretry_ > N_RETRY_ATTEMPTS)
        {
            logging::get_logger()->critical("Max retry attempts reached. Exiting...");
            exit(1);
        }
        reconnect();
    }
}

void callback::on_failure(const mqtt::token &tok)
{
    logging::get_logger()->error("Connection attempt failed");
    if (++nretry_ > N_RETRY_ATTEMPTS)
    {
        logging::get_logger()->critical("Max retry attempts reached. Exiting...");
        exit(1);
    }
    reconnect();
}

void callback::on_success(const mqtt::token &tok) {}

void callback::connected(const std::string &cause)
{
    logging::get_logger()->info("\nConnection success");
    logging::get_logger()->info("\nSubscribing to topic '{}' for client {} using QoS{}\n", TOPICCMD, CLIENT_ID, QOS);
    logging::get_logger()->info("Press Q<Enter> to quit\n");

    cli_.subscribe(TOPICCMD, QOS, nullptr, subListener_);
    cli_.subscribe(TOPICBEIDOU1, QOS, nullptr, subListener_);
    cli_.subscribe(TOPICBEIDOU2, QOS, nullptr, subListener_);
}

void callback::connection_lost(const std::string &cause)
{
    logging::get_logger()->warn("\nConnection lost");
    if (!cause.empty())
        logging::get_logger()->warn("\tcause: {}", cause);

    logging::get_logger()->info("Reconnecting...");
    nretry_ = 0;
    reconnect();
}

void callback::message_arrived(mqtt::const_message_ptr msg)
{
    logging::get_logger()->info("Message arrived");
    logging::get_logger()->info("\ttopic: '{}'", msg->get_topic());
    logging::get_logger()->info("\tpayload: '{}'", msg->to_string());

    std::string json_str = msg->to_string();

    if (TOPICBEIDOU1 == msg->get_topic())
    {
        // logging::get_logger()->info("this is topic1");
        // logging::get_logger()->info(json_str);
        BeidouModule::Instance()->setRawMsg(json_str, 0);
        Position pos;
        if (BeidouModule::Instance()->get_position(0, pos))
        {
            logging::get_logger()->info(pos.latitude);
            logging::get_logger()->info(pos.longitude);
        }
        else
            logging::get_logger()->info("data illegal");
    }
    else if (TOPICBEIDOU2 == msg->get_topic())
    {
        // logging::get_logger()->info("this is topic2");
        // logging::get_logger()->info(json_str);
        BeidouModule::Instance()->setRawMsg(json_str, 1);
    }
    else if (TOPICCMD == msg->get_topic())
    {
        logging::get_logger()->info("hhhhhhhhhhhhhhhhhhhhhhhhh");
        std::thread t([json_str]()
                      { parse_json_message(json_str); });
        t.detach();
        logging::get_logger()->info("11111222222222233333333344444444444444444444444444444");
    }
}

void callback::delivery_complete(mqtt::delivery_token_ptr token)
{
    logging::get_logger()->info("Delivery complete for token: [{}]", token->get_message_id());
}

int connectToDrone(Mavsdk &g_mavsdk, const std::string &connection_url)
{
    logging::get_logger()->info("Connecting to {}", connection_url);
    ConnectionResult connection_result = g_mavsdk.add_any_connection(connection_url);
    if (connection_result != ConnectionResult::Success)
    {
        logging::get_logger()->error("Connection PX4 failed");
        return -1;
    }
    return 0;
}

int main(int argc, char *argv[])
{

#ifdef REAL_HARDWARE
    std::string connection_url = "serial:///dev/ttyACM0:921600";
#elif defined(SIMULATION)
    std::string connection_url = "udpin://0.0.0.0:14540";
    GazeboCamera::Instance()->init(argc, argv, subscribePtr);
#else
    logging::get_logger()->error("mode error");
#endif

    int connect_status = connectToDrone(g_mavsdk, connection_url);
    if (connect_status != 0)
    {
        logging::get_logger()->error("connect_status error");
    }

    auto system = g_mavsdk.first_autopilot(3.0);
    if (!system)
    {
        logging::get_logger()->error("Timed out waiting for system");
        return 1;
    }

    g_action.emplace(system.value());
    g_mission.emplace(system.value());
    g_telemetry.emplace(system.value());
    g_mavlinkpassthrough.emplace(system.value());
    g_offboard.emplace(system.value());
    g_camera.emplace(system.value());

    auto serverURI = (argc > 1) ? std::string{argv[1]} : DFLT_SERVER_URI;

    mqtt::async_client cli(serverURI, CLIENT_ID);

    mqtt::connect_options connOpts;
    connOpts.set_clean_session(false);
    connOpts.set_keep_alive_interval(std::chrono::seconds(300));

    connOpts.set_user_name(MQTTUSER);    // ← 添加用户名
    connOpts.set_password(MQTTPASSWORD); // ← 添加密码

    callback cb(cli, connOpts);
    cli.set_callback(cb);

    try
    {
        logging::get_logger()->info("Connecting to the MQTT server '{}'...", serverURI);
        cli.connect(connOpts, nullptr, cb);
    }
    catch (const mqtt::exception &exc)
    {
        logging::get_logger()->error("ERROR: Unable to connect to MQTT server: '{}' {}", serverURI, exc.what());
        return 1;
    }

    while (std::tolower(std::cin.get()) != 'q')
        ;

    try
    {
        logging::get_logger()->info("\nDisconnecting from the MQTT server...");
        cli.disconnect()->wait();
        logging::get_logger()->info("OK");
    }
    catch (const mqtt::exception &exc)
    {
        logging::get_logger()->error(exc.what());
        return 1;
    }

    return 0;
}