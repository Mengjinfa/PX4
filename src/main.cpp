#include "flight_procedure.hpp"
#include "fly_mission.hpp"
#include "landing_state_machine.hpp"
#include "mavsdk_members.hpp"
#include "mqtt_client.hpp"
#include "pid.hpp"
#include "telemetry_monitor.hpp"
#include "user_task.hpp"

#include <chrono>
#include <fcntl.h>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <optional>
#include <thread> // 用于多线程支持
#include <unistd.h>

using namespace std;
using namespace mavsdk;

int main()
{
    // 程序运行标志
    std::atomic<bool> running(true);

    /*::::::::::::::::::::::::::::: MQTT 初始化与启动 ::::::::::::::::::::::::::::::*/
    Mqtt mqtt;                                      // mqtt接收器
    mqtt.init();                                    // MQTT初始化
    mqtt.subscribeTopic("test", handleTestMessage); // 订阅test主题

    /*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    std::string connection_url = "udpin://0.0.0.0:14540";                  // 仿真环境通过UDP连接
    Mavsdk drone_sdk{Mavsdk::Configuration{ComponentType::GroundStation}}; // 创建MAVSDK实例并配置为地面站类型

    // 建立与无人机的通信连接
    drone_sdk.add_any_connection(connection_url);

    // 等待自动飞行器系统出现（最多等待10秒）
    auto system = drone_sdk.first_autopilot(10.0);
    if (!system) // 超时处理
    {
        mqtt.sendMessage(REPLAY_TOPIC, "无人机连接等待超时");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        return 1;
    }

    // 创建各种无人机功能模块的可选实例
    std::optional<MavlinkPassthrough> g_mavlinkpassthrough;
    std::optional<MissionRaw> g_mission_raw;
    std::optional<Telemetry> g_telemetry;
    std::optional<Offboard> g_offboard;
    std::optional<Mission> g_mission;
    std::optional<Action> g_action;
    std::optional<Camera> g_camera;

    // 如果连接到飞控系统，初始化所有功能模块
    if (system.has_value()) // 检查 optional 对象是否包含一个有效值。
    {
        mqtt.sendMessage(REPLAY_TOPIC, "无人机连接成功");

        auto &system_ref = system.value();

        g_mavlinkpassthrough.emplace(system_ref);
        g_mission_raw.emplace(system_ref);
        g_telemetry.emplace(system_ref);
        g_offboard.emplace(system_ref);
        g_mission.emplace(system_ref);
        g_action.emplace(system_ref);
        g_camera.emplace(system_ref);
    }
    else
    {
        mqtt.sendMessage(REPLAY_TOPIC, "无人机连接失败");
    }

    // 创建Mavsdk对象，整合所有MAVSDK功能模块
    // 该对象用于在整个程序中传递和共享无人机相关功能
    Mavsdk_members mavsdk{
        g_mavlinkpassthrough.value(),
        g_mission_raw.value(),
        g_telemetry.value(),
        g_offboard.value(),
        g_mission.value(),
        g_action.value(),
        g_camera.value()};

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
    AprilTagTracker tag_tracker;                   // AprilTag跟踪器实例
    LandingStateMachine landingstate_machine;      // 状态机实例
    PID pid;                                       // PID控制器实例
    Telemetry &telemetry = mavsdk.telemetry;       // 获取遥测数据模块引用
    TelemetryMonitor telemetry_monitor(telemetry); // 创建遥测监控器实例

    // 启动AprilTag跟踪器
    // tag_tracker.start(argc, argv);

    // 起飞并解锁无人机，起飞高度为5米
    // arming_and_takeoff(mavsdk, 5.0);

    /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

    while (running)
    {
        AprilTagData landmark = tag_tracker.getData();                       // 获取最新AprilTag检测结果
        PIDOutput PID_out = pid.Output_PID();                                // 更新PID结果
        LandingState state_ = landingstate_machine.getCurrentStateMachine(); // 输出状态机处于的模式

        Telemetry::PositionNed current_position = telemetry_monitor.getCurrentPosition(); // 获取当前无人机位置(NED坐标系)
        Telemetry::EulerAngle euler_angle = telemetry_monitor.getCurrentEulerAngles();    // 获取当前欧拉角姿态
        Telemetry::FlightMode flight_mode = telemetry_monitor.getCurrentFlightMode();     // 获取当前飞行模式
        Telemetry::RawGps gps_raw = telemetry_monitor.getCurrentRawGps();                 // 获取当前GPS信息

        float current_relative_altitude_m = telemetry_monitor.getCurrentRelativeAltitudeM(); // 获取当前相对高度
        float current_distance_sensor_m = telemetry_monitor.getCurrentDistanceSensorM();     // 获取当前距离传感器高度

        pid.getLandmark(landmark); // 获取地标检测数据
        pid.PID_update();          // 更新PID控制器状态

        // landingstate_machine.StartStateMachine(current_position, current_relative_altitude_m, euler_angle.yaw_deg); // 启动状态机
        landingstate_machine.getLandmark(landmark); // 获取地标检测数据
        landingstate_machine.getPIDOut(PID_out);    // 获取PID计算结果
        landingstate_machine.updateState(mavsdk);   // 更新状态机状态

        user_task(mavsdk);

        /*:::::::::::::::::::::::::::::::::::::::::::::::: 日志（每秒一次） ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
        static auto lastWriteTime = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - lastWriteTime).count() >= 1)
        {
            // 发送的消息 - 字符串
            std::string logMessage = "Mode: " + telemetry_monitor.flight_mode_str(flight_mode) + ", state : " + landingstate_machine.landingStateToString(state_) + "\n";
            logMessage += "Landmark:(x: " + std::to_string(landmark.x) + ", y: " + std::to_string(landmark.y) + ")" + "\n";
            logMessage += "err:(x: " + std::to_string(landmark.err_x) + ", y: " + std::to_string(landmark.err_y) + ")" + "\n";
            logMessage += "PID:(x: " + std::to_string(PID_out.x) + ", y: " + std::to_string(PID_out.y) + ")" + "\n";
            logMessage += "Euler:(yaw: " + std::to_string(euler_angle.yaw_deg) + ", pitch: " + std::to_string(euler_angle.pitch_deg) + ", roll: " + std::to_string(euler_angle.roll_deg) + ")" + "\n";
            logMessage += "Position:(x: " + std::to_string(current_position.north_m) + ", y: " + std::to_string(current_position.east_m) + ", z: " + std::to_string(-current_position.down_m) + ")" + "\n";
            logMessage += "GPS:(x: " + std::to_string(gps_raw.latitude_deg) + ", y: " + std::to_string(gps_raw.longitude_deg) + ")" + "\n";
            logMessage += "distance: " + std::to_string(current_distance_sensor_m) + "\n";

            mqtt.sendMessage(REPLAY_TOPIC, logMessage); // 发送MQTT消息 发送到flight_tx主题

            lastWriteTime = std::chrono::steady_clock::now(); // 更新时间
        }
        /*:::::::::::::::::::::::::::::::::::::::::::::::: 日志（每秒一次） ::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 间歇休眠减少CPU占用(10Hz)
    }

    // tag_tracker.stop(); // 停止AprilTag跟踪器

    return 0;
}