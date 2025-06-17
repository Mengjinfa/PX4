#include "async_mqtt.hpp"                   // 引入异步 MQTT 通信模块，实现异步消息订阅与发布
#include "BeiDouModule.h"                   // 引入北斗定位模块，处理北斗卫星导航数据
#include "logger.hpp"                       // 引入全局日志模块，用于记录程序运行状态和调试信息
#include "rc.hpp"                           // 引入遥控模块，处理遥控器输入
#include "sim_camera_module.h"              // 引入仿真相机模块，用于 Gazebo 仿真环境
#include "takeoff_and_land.hpp"             // 引入起降模块，实现无人机起飞与降落控制
#include "target_tracker.hpp"               // 引入目标跟踪模块，跟踪特定目标
#include "vision_guided_mission_flying.hpp" // 引入视觉引导任务飞行模块

#include <mavsdk/mavsdk.h>                                          // 引入 MAVSDK 核心库，提供与无人机通信的接口
#include <mavsdk/plugins/action/action.h>                           // 引入动作控制插件（起飞、降落等）
#include <mavsdk/plugins/camera/camera.h>                           // 引入相机控制插件
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h> // 引入 MAVLink 透传插件
#include <mavsdk/plugins/mission/mission.h>                         // 引入任务规划插件（航点任务）
#include <mavsdk/plugins/offboard/offboard.h>                       // 引入离板控制插件（自主飞行）
#include <mavsdk/plugins/telemetry/telemetry.h>                     // 引入遥测数据插件（位置、高度等）

#include <atomic>            // 引入原子操作支持，确保多线程环境下变量访问的线程安全
#include <fcntl.h>           // 引入文件控制头文件
#include <fstream>           // 引入文件流操作
#include <nlohmann/json.hpp> // 引入 JSON 解析库，处理消息格式
#include <thread>            // 引入线程支持
#include <unistd.h>          // 引入 Unix 标准库头文件，提供系统调用接口

using namespace mavsdk; // 使用 MAVSDK 命名空间，避免重复书写

// 全局变量定义
std::string subscribePtr = "/gazebo/default/iris/base_link/camera/image"; // Gazebo 仿真中相机图像的订阅主题，用于获取仿真环境中的视觉数据

// 任务控制标志（原子变量，确保线程安全）
std::atomic<int> start_detect{0};           // 启动目标检测标志
std::atomic<int> find_landmark{0};          // 寻找地标标志
std::atomic<int> start_posctl{0};           // 启动位置控制标志
std::atomic<int> is_take_off{0};            // 起降状态标志
std::atomic<bool> detection_running{false}; // 检测运行状态标志

// 全局 MAVSDK 实例及各功能插件
Mavsdk g_mavsdk{Mavsdk::Configuration{ComponentType::GroundStation}}; // 初始化 MAVSDK 实例，配置为地面站类型

// 以下为 MAVSDK 各功能插件的可选实例（optional），使用时需确认已初始化
std::optional<Action> g_action;                         // 动作控制插件（起飞、降落等）
std::optional<Mission> g_mission;                       // 任务规划插件（航点任务）
std::optional<Telemetry> g_telemetry;                   // 遥测数据插件（获取位置、高度等）
std::optional<MavlinkPassthrough> g_mavlinkpassthrough; // MAVLink 透传插件
std::optional<Offboard> g_offboard;                     // 离板控制插件（自主飞行模式）
std::optional<Camera> g_camera;                         // 相机控制插件

/**
 * @brief MQTT 动作监听器的失败回调函数
 *
 * 当 MQTT 操作失败时调用，记录错误信息并输出相关令牌信息。
 *
 * @param tok 触发回调的 MQTT 令牌
 */
void action_listener::on_failure(const mqtt::token &tok)
{
    logging::get_logger()->error("{} failure", name_);
    if (tok.get_message_id() != 0)
        logging::get_logger()->error(" for token: [{}]", tok.get_message_id());
}

/**
 * @brief MQTT 动作监听器的成功回调函数
 *
 * 当 MQTT 操作成功时调用，记录成功信息并输出相关令牌信息。
 *
 * @param tok 触发回调的 MQTT 令牌
 */
void action_listener::on_success(const mqtt::token &tok)
{
    logging::get_logger()->info("{} success", name_);

    if (tok.get_message_id() != 0)
        logging::get_logger()->info(" for token: [{}]", tok.get_message_id());

    auto top = tok.get_topics();

    if (top && !top->empty())
        logging::get_logger()->info("\ttoken topic: '{}', ...", (*top)[0]);
}

/**
 * @brief 处理启动任务的函数
 *
 * 根据传入的 missionId 更新相应的任务控制标志。
 *
 * @param missionId 任务标识符
 * @return int 返回处理结果，0 表示成功
 */
int process_start_mission(const std::string &missionId)
{
    if (missionId == "1")
    {
        start_detect.store(start_detect.load() ? 0 : 1); // 视觉检测状态
        logging::get_logger()->info("start_detect: {}", start_detect.load());
        return 0;
    }
    else if (missionId == "21")
    {
        find_landmark.store(find_landmark.load() ? 0 : 1); // 地标识别状态
        logging::get_logger()->info("find_landmark: {}", find_landmark.load());
        return 0;
    }
    else if (missionId == "2")
    {
        start_posctl.store(start_posctl.load() ? 0 : 1); // 位置控制状态
        logging::get_logger()->info("start_posctl: {}", start_posctl.load());
        return 0;
    }
    else if (missionId == "3")
    {
        is_take_off.store(is_take_off.load() ? 0 : 1); // 起飞状态
        logging::get_logger()->info("is_take_off: {}", is_take_off.load());
        return 0;
    }
    else if (missionId == "4")
    {
        detection_running.store(detection_running.load() ? false : true); // 目标检测进程状态
        logging::get_logger()->info("detection_running: {}", detection_running.load());
        return 0;
    }

    // 如果 missionId 不匹配上述预设值，则视为完整任务ID，执行完整飞行任务
    int reply_code = fly_mission(missionId,
                                 g_action.value(),
                                 g_mission.value(),
                                 g_telemetry.value(),
                                 g_offboard.value(),
                                 g_camera.value(),
                                 g_mavlinkpassthrough.value());

    return reply_code;
}

/**
 * @brief 处理启动任务的 JSON 消息
 *
 * 解析传入的 JSON 字符串，提取 missionId 并调用 process_start_mission 处理。
 *
 * @param json_str 传入的 JSON 字符串
 */
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

/**
 * @brief 处理起飞和降落命令的函数
 *
 * 根据传入的 enable 值调用 takeoff_and_land 函数执行起飞或降落。
 *
 * @param enable 启用或禁用起飞/降落，1 表示启用，0 表示禁用
 * @return int 返回处理结果，0 表示成功
 */
int process_takeoff_land(int enable)
{
    logging::get_logger()->info("777777777777777");
    int reply_code = takeoff_and_land(enable, g_action.value(), g_telemetry.value(), g_offboard.value());
    return reply_code;
}

/**
 * @brief 处理起飞和降落的 JSON 消息
 *
 * 解析传入的 JSON 字符串，提取 enable 值并调用 process_takeoff_land 处理。
 *
 * @param json_str 传入的 JSON 字符串
 */
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

/**
 * @brief 处理启用自动着陆的函数
 *
 * 设置遥测数据订阅频率，并订阅位置数据以监控高度。
 *
 * @param enable 启用或禁用自动着陆，1 表示启用，0 表示禁用
 * @return int 返回处理结果，0 表示成功
 */
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
    auto position_handle = g_telemetry.value().subscribe_position([](Telemetry::Position position)
                                                                  { logging::get_logger()->info("当前高度zxkkkk111111：{}", position.relative_altitude_m); });
    return 0;
}

/**
 * @brief 处理启用自动着陆的 JSON 消息
 *
 * 解析传入的 JSON 字符串，提取 enable 值并调用 process_enable_auto_landing 处理。
 *
 * @param json_str 传入的 JSON 字符串
 */
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

/**
 * @brief 处理启动目标检测的函数
 *
 * 根据传入的 enable 值更新检测运行状态标志。
 *
 * @param enable 启用或禁用目标检测，1 表示启用，0 表示禁用
 * @return int 返回处理结果，0 表示成功
 */
int process_start_detect(int enable)
{
    return 0;
}

/**
 * @brief 处理启动目标检测的 JSON 消息
 *
 * 解析传入的 JSON 字符串，提取 enable 值并调用 process_start_detect 处理。
 *
 * @param json_str 传入的 JSON 字符串
 */
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

/**
 * @brief 处理上传任务的 JSON 消息
 *
 * 目前该函数为空，需要根据具体需求实现上传任务的功能。
 *
 * @param json_str 传入的 JSON 字符串
 */
void handle_upload_mission(const std::string &json_str) {}

/**
 * @brief 处理命令的函数
 *
 * 解析传入的 JSON 字符串，根据 method 字段调用相应的处理函数。
 *
 * @param json 传入的 JSON 字符串
 */
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

/**
 * @brief 解析 JSON 消息并处理命令
 *
 * 调用 process_command 函数处理传入的 JSON 字符串。
 *
 * @param json_string 传入的 JSON 字符串
 */
void parse_json_message(const std::string &json_string)
{
    process_command(json_string);
}

/**
 * @brief MQTT 回调类的构造函数
 *
 * 初始化 MQTT 回调对象，设置重试次数、客户端实例、连接选项和订阅监听器。
 *
 * @param cli MQTT 客户端实例
 * @param connOpts 连接选项
 */
callback::callback(mqtt::async_client &cli, mqtt::connect_options &connOpts)
    : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription") {}

/**
 * @brief 重新连接 MQTT 服务器
 *
 * 实现 MQTT 重新连接逻辑，包括等待一段时间后尝试重新连接。
 *
 * 如果超过最大重试次数，则记录关键错误并退出程序。
 */
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

/**
 * @brief MQTT 连接失败回调
 *
 * 当连接尝试失败时调用，记录错误信息并尝试重新连接。
 */
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

/**
 * @brief MQTT 连接成功回调
 *
 * 当连接成功时调用，记录成功信息并订阅相关主题。
 */
void callback::connected(const std::string &cause)
{
    logging::get_logger()->info("\nConnection success");
    logging::get_logger()->info("\nSubscribing to topic '{}' for client {} using QoS{}\n", TOPICCMD, CLIENT_ID, QOS);
    logging::get_logger()->info("Press Q<Enter> to quit\n");

    cli_.subscribe(TOPICCMD, QOS, nullptr, subListener_);
    cli_.subscribe(TOPICBEIDOU1, QOS, nullptr, subListener_);
    cli_.subscribe(TOPICBEIDOU2, QOS, nullptr, subListener_);
}

/**
 * @brief MQTT 连接丢失回调
 *
 * 当连接丢失时调用，记录警告信息并尝试重新连接。
 */
void callback::connection_lost(const std::string &cause)
{
    logging::get_logger()->warn("\nConnection lost");
    if (!cause.empty())
        logging::get_logger()->warn("\tcause: {}", cause);

    logging::get_logger()->info("Reconnecting...");
    nretry_ = 0;
    reconnect();
}

/**
 * @brief MQTT 消息到达回调
 *
 * 当接收到消息时调用，记录消息主题和负载，并处理消息内容。
 *
 * @param msg 接收到的 MQTT 消息
 */
void callback::message_arrived(mqtt::const_message_ptr msg)
{
    logging::get_logger()->info("Message arrived");
    logging::get_logger()->info("\ttopic: '{}'", msg->get_topic());
    logging::get_logger()->info("\tpayload: '{}'", msg->to_string());

    std::string json_str = msg->to_string();

    if (TOPICBEIDOU1 == msg->get_topic())
    {
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

/**
 * @brief MQTT 消息发送完成回调
 *
 * 当消息发送完成时调用，记录发送完成的信息。
 *
 * @param token 发送完成的 MQTT 令牌
 */
void callback::delivery_complete(mqtt::delivery_token_ptr token)
{
    logging::get_logger()->info("Delivery complete for token: [{}]", token->get_message_id());
}

/**
 * @brief 连接无人机函数
 *
 * 根据连接 URL 连接无人机，配置为 UDP 或串口连接。
 *
 * @param g_mavsdk MAVSDK 实例
 * @param connection_url 连接 URL
 * @return int 返回连接结果，0 表示成功
 */
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

/**
 * @brief 主函数
 *
 * 主要流程包括：
 * 1. 根据编译选项选择连接方式（真实硬件或仿真）
 * 2. 连接无人机
 * 3. 初始化 MAVSDK 插件实例
 * 4. 连接 MQTT 服务器
 * 5. 进入消息循环，等待用户输入 'q' 退出
 *
 * @param argc 参数个数
 * @param argv 参数数组
 * @return int 返回程序退出状态
 */
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