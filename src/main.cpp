#include "async_mqtt.hpp"
#include "flight_procedure.hpp"
#include "mqtt_client.hpp"
#include "pid.hpp"
#include "state_machine.hpp"

#include <chrono>
#include <fcntl.h>
#include <fstream>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <thread> // 用于多线程支持
#include <unistd.h>

namespace fs = std::filesystem;
using namespace std;
using namespace mavsdk;

// 辅助函数：将飞行模式枚举转换为字符串
std::string telemetry_flight_mode_str(Telemetry::FlightMode mode)
{
    switch (mode)
    {
        case Telemetry::FlightMode::Unknown:
            return "Unknown";
        case Telemetry::FlightMode::Ready:
            return "Ready";
        case Telemetry::FlightMode::Takeoff:
            return "Takeoff";
        case Telemetry::FlightMode::Hold:
            return "Hold";
        case Telemetry::FlightMode::Mission:
            return "Mission";
        case Telemetry::FlightMode::ReturnToLaunch:
            return "ReturnToLaunch";
        case Telemetry::FlightMode::Land:
            return "Land";
        case Telemetry::FlightMode::Offboard:
            return "Offboard";
        case Telemetry::FlightMode::FollowMe:
            return "FollowMe";
        case Telemetry::FlightMode::Posctl:
            return "Position";
        case Telemetry::FlightMode::Altctl:
            return "Altitude";
        case Telemetry::FlightMode::Stabilized:
            return "Stabilized";
        case Telemetry::FlightMode::Acro:
            return "Acro";
        default:
            return "Invalid";
    }
}

int main(int argc, char *argv[])
{
    // 程序运行标志
    std::atomic<bool> running(true);

    /*::::::::::::::::::::::::::::: MQTT 初始化与启动 ::::::::::::::::::::::::::::::*/
    const std::string saveDir = "/home/senen/桌面/receive"; // 航线文件保存路径
    Mqtt mqtt(saveDir);                                     // mqtt接收器
    mqtt.init();                                            // MQTT初始化

    // 启动监听航线文件MQTT消息（独立线程，避免阻塞ROS主循环）
    // 创建了一个新的线程 mqtt_listen_thread，该线程会异步执行 mqtt.Monitor_flight_file() 方法。
    // std::thread mqtt_listen_thread([&mqtt]()
    //                               { mqtt.Monitor_flight_file(); });

    mqtt.subscribeTopic("test"); // 订阅test主题

    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    std::string connection_url = "udpin://0.0.0.0:14540";                  // 仿真环境通过UDP连接
    Mavsdk drone_sdk{Mavsdk::Configuration{ComponentType::GroundStation}}; // 创建MAVSDK实例并配置为地面站类型

    // 建立与无人机的通信连接
    drone_sdk.add_any_connection(connection_url);

    // 等待自动飞行器系统出现（最多等待10秒）
    // first_autopilot() 方法会阻塞，直到找到第一个自动飞行器系统或超时。
    // 如果在10秒内找到了自动飞行器系统，返回一个 std::optional<Autopilot> 对象。
    // 如果在10秒内没有找到自动飞行器系统，返回一个空的 std::optional<Autopilot> 对象。
    // 该方法的作用是等待无人机系统连接成功，并返回第一个找到的自动飞行器系统实例。
    // 这通常用于在无人机启动后，等待其飞行控制系统准备就绪，以便后续进行通信和控制。
    auto system = drone_sdk.first_autopilot(10.0);
    if (!system) // 超时处理
    {
        mqtt.sendMessage(REPLAY_TOPIC, "无人机连接等待超时");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        return 1;
    }

    // 创建各种无人机功能模块的可选实例
    // 声明了一个全局变量 g_mavlinkpassthrough，它的类型是 std::optional<MavlinkPassthrough>
    // std::optional 用于表示一个可能存在的值，类似于一个轻量级的容器，要么包含一个值，要么为空（std::nullopt）
    std::optional<MavlinkPassthrough> g_mavlinkpassthrough; // MAVLink透传
    std::optional<MissionRaw> g_mission_raw;                // 航线任务接口
    std::optional<Telemetry> g_telemetry;                   // 遥测数据
    std::optional<Offboard> g_offboard;                     // 外部控制模式
    std::optional<Mission> g_mission;                       // 航线任务规划
    std::optional<Action> g_action;                         // 飞行控制动作（起飞、降落等）
    std::optional<Camera> g_camera;                         // 相机控制

    // 如果连接到飞控系统，初始化所有功能模块
    if (system.has_value()) // 检查 optional 对象是否包含一个有效值。
    {
        // 系统连接成功
        mqtt.sendMessage(REPLAY_TOPIC, "无人机连接成功");

        // 获取已发现的无人机系统的引用（使用value()方法获取optional中的值）
        auto &system_ref = system.value();

        // 使用emplace方法直接在 optional 内部构造对象，避免不必要的拷贝或移动操作
        // 如果 optional 为空：直接在其内部空间构造对象
        // 如果 optional 已有值：先销毁当前值，再构造新值
        g_mavlinkpassthrough.emplace(system_ref); // 初始化MAVLink透传插件（用于发送自定义MAV// 辅助函数：将飞行模式枚举转换为字符串
        g_mission_raw.emplace(system_ref);        // 初始化原始任务插件（用于处理MAVLink原生任务格式）
        g_telemetry.emplace(system_ref);          // 初始化遥测数据插件（用于获取无人机状态信息）
        g_offboard.emplace(system_ref);           // 初始化外部控制插件（用于发送实时控制命令）
        g_mission.emplace(system_ref);            // 初始化任务规划插件（用于加载和执行预定义任务）
        g_action.emplace(system_ref);             // 初始化飞行控制插件（用于起飞、降落、悬停等操作）
        g_camera.emplace(system_ref);             // 初始化相机控制插件（用于控制无人机相机）
    }
    else
    {
        mqtt.sendMessage(REPLAY_TOPIC, "无人机连接失败");
    }

    // 创建Mavsdk对象，整合所有MAVSDK功能模块
    // 该对象用于在整个程序中传递和共享无人机相关功能
    Mavsdk_members mavsdk{
        g_mavlinkpassthrough.value(), // MAVLink透传模块：用于发送和接收原始MAVLink消息
        g_mission_raw.value(),        // 原始任务模块：提供对MAVLink原生任务格式的支持
        g_telemetry.value(),          // 遥测数据模块：提供无人机状态信息（位置、姿态、速度等）
        g_offboard.value(),           // 外部控制模块：用于发送实时控制命令（如位置、速度指令）
        g_mission.value(),            // 任务规划模块：用于加载、执行和管理预定义的飞行任务
        g_action.value(),             // 飞行控制动作模块：用于起飞、降落、悬停等基本飞行操作
        g_camera.value()              // 相机控制模块：用于控制无人机搭载的相机（拍照、录像等）
    };

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    AprilTagTracker tag_tracker;             // AprilTag跟踪器
    StateMachine state_machine;              // 状态机
    PID pid;                                 // PID控制器
    Telemetry &telemetry = mavsdk.telemetry; // 获取遥测数据模块引用
    Telemetry::FlightMode flight_mode;       // 飞行模式

    // 启动AprilTag跟踪器
    tag_tracker.start(argc, argv);

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

    while (running)
    {
        AprilTagData landmark = tag_tracker.getData();         // 获取最新AprilTag检测结果
        PIDOutput PID_out = pid.Output_PID();                  // 更新PID结果
        LandingState state_ = state_machine.getCurrentState(); // 输出状态机处于的模式

        pid.getLandmark(landmark); // 获取地标检测数据

        state_machine.StartStateMachine(mavsdk); // 启动状态机
        state_machine.getLandmark(landmark);     // 获取地标检测数据
        state_machine.getPIDOut(PID_out);        // 获取PID计算结果

        // 注册飞行模式回调函数
        telemetry.subscribe_flight_mode([](Telemetry::FlightMode flight_mode)
                                        { std::cout << "当前飞行模式: " << telemetry_flight_mode_str(flight_mode) << std::endl; });

        /*:::::::::::::::::::::::::::::::::::::::::::::::: 日志（每秒一次） ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
        static auto lastWriteTime = std::chrono::steady_clock::now(); // 上次写入时间
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - lastWriteTime).count() >= 1)
        {
            // 发送的消息 - 字符串
            std::string logMessage = "Mode: " + telemetry_flight_mode_str(flight_mode) + ", state : " + state_machine.landingStateToString(state_) + "\n ";
            logMessage += "Landmark:(x: " + std::to_string(landmark.x) + ", y: " + std::to_string(landmark.y) + ")" + "\n";
            logMessage += "err:(x: " + std::to_string(landmark.err_x) + ", y: " + std::to_string(landmark.err_y) + ")" + "\n";

            mqtt.sendMessage(REPLAY_TOPIC, logMessage); // 发送MQTT消息 发送到flight_tx主题

            lastWriteTime = std::chrono::steady_clock::now(); // 更新时间
        }
        /*:::::::::::::::::::::::::::::::::::::::::::::::: 日志（每秒一次） ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 间歇休眠减少CPU占用(10Hz)
    }

    // mqtt_listen_thread.join(); // 等待MQTT线程结束
    tag_tracker.stop(); // 停止AprilTag跟踪器

    return 0;
}