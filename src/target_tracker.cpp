#include "apriltag_tracker.hpp"
#include "detection_state_machine.hpp"
#include "drone_controller.hpp"
#include "logger.hpp"
#include "mqtt_client.hpp"
#include "sim_camera_module.hpp"
#include "telemetry_monitor.hpp"

#include <algorithm>
#include <apriltag/apriltag.h>
#include <apriltag/tag25h9.h>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <fcntl.h>
#include <iostream>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <queue>
#include <sys/stat.h>
#include <termios.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

using namespace mavsdk;
using namespace cv;

std::atomic<bool> detection_running; // 识别降落控制线程 // 可借由mqtt的消息中断 offboard模式 避免危险模式
std::condition_variable queue_cond;  // 条件变量，用于线程同步
std::queue<cv::Mat> image_queue;     // 图像缓冲区队列
std::mutex queue_mutex;              // 队列互斥锁

/**
 * @brief 目标检测主线程函数
 *
 * 实现无人机目标检测与视觉引导控制逻辑， 包括AprilTag检测、状态机更新、无人机控制等。
 *
 * @param context 无人机控制相关组件
 */
void detection_thread(Mavsdk_members &mavsdk)
{
    // 提取无人机控制相关组件
    Telemetry &telemetry = mavsdk.telemetry;
    Offboard &offboard = mavsdk.offboard;
    // mqtt_client::publish(PX4_REPLY, "开始检测");

    // 初始化检测与控制组件
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待系统初始化
    DroneController drone_controller(0.2f, 0.5f);         // 无人机控制器（用于目标跟踪与控制）并设置最大速度为0.2米/秒，绕圆半径为0.5米
    // TelemetryMonitor monitor(telemetry, 0.5f);            // 遥测监控器（用于获取无人机位置与高度）并设置降落阈值为0.5米
    DetectionStateMachine state_machine; // 状态机（用于管理无人机状态与行为）
    AprilTagTracker tracker;             // AprilTag跟踪器（用于处理图像帧与检测AprilTag）

    // 启动AprilTag线程
    // tracker.start();

    // // 启动无人机Offboard模式
    // Offboard::VelocityBodyYawspeed zero_vel{}; // 创建一个零速度的Offboard命令
    // offboard.set_velocity_body(zero_vel);      // 发布零速度命令激活Offboard模式下稳定悬停
    // offboard.start();                          // 启动Offboard模式

    // // 检查Offboard模式是否激活
    // if (!offboard.is_active())
    // {
    //     logging::get_logger()->error("Offboard mode not active!");
    //     return;
    // }

    // 初始化位置与时间参数
    auto no_search_start = std::chrono::steady_clock::now();
    auto current_position = monitor.getCurrentPosition();
    float current_relative_altitude_m = monitor.getCurrentRelativeAltitudeM();
    auto start_position = current_position;

    // 打印初始位置与高度
    double zxkx = 0;
    double zxky = 0;
    static auto lastWriteTime = std::chrono::steady_clock::now(); // 上次写入时间
    std::string gloalstaus = "空";

    // 主循环：检测-控制-状态更新
    while (detection_running && !monitor.hasLanded())
    {
        // 提取并打印遥测数据（位置误差、飞行模式、高度等）
        auto zxkmode = telemetry.flight_mode();
        float zxkhight = current_relative_altitude_m;

        std::ostringstream oss;                    // 创建字符串流用于格式化输出
        oss << std::fixed << std::setprecision(2); // 设置输出格式为小数点后两位
        oss << "status:" << gloalstaus << "\n"
            << "error_x:" << zxkx << "\n"
            << "error_y:" << zxky << "\n"
            << "mode:" << zxkmode << "\n"
            << "altitude:" << zxkhight << "\n";

        // 每秒通过MQTT发布一次遥测数据
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - lastWriteTime).count() >= 1)
        {
            std::cout << oss.str() << std::endl; // 打印到控制台
            std::string data = oss.str();        // 将数据转换为字符串
            // mqtt_client::publish(PX4_REPLY, data);            // 发布到MQTT主题
            lastWriteTime = std::chrono::steady_clock::now(); // 更新时间
        }

        // 执行AprilTag检测并更新状态机
        AprilTagData result = tracker.getData();                // 执行AprilTag检测，获取检测结果
        DroneState state = state_machine.update(result.iffind); // 根据检测结果更新状态机，获取新的无人机状态

        current_position = monitor.getCurrentPosition();                       // 获取无人机当前位置
        current_relative_altitude_m = monitor.getCurrentRelativeAltitudeM();   // 获取无人机当前相对高度
        state_machine.setCurrentRelativeAltitude(current_relative_altitude_m); // 将当前高度传递给状态机，用于状态决策

        // 根据状态机输出执行不同策略
        if (state == DroneState::TRACKING && result.iffind)
        {
            // 目标跟踪状态：计算偏差并控制无人机对准目标
            gloalstaus = "检测到目标";

            double dx = (result.x - result.width / 2.0) / result.width;
            double dy = (result.y - result.height / 2.0) / result.height;
            zxkx = dx;
            zxky = dy;

            // 通过无人机控制器的trackTarget方法来调整
            // 传入偏差、板外控制插件、当前无人机位置和相对高度
            drone_controller.trackTarget(result.norm_err_x, result.norm_err_y, offboard, current_position, current_relative_altitude_m);
        }
        else if (state == DroneState::SEARCHING)
        {
            // 搜索状态：保持高度并等待目标出现
            gloalstaus = "正在查找";

            Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = current_position.north_m;
            setpoint.east_m = current_position.east_m;
            setpoint.down_m = -(state_machine.getNotDetectedAltitude());
            setpoint.yaw_deg = 0.0f;

            offboard.set_position_ned(setpoint);

            no_search_start = std::chrono::steady_clock::now();
        }
        else if (state == DroneState::NOT_DETECTED)
        {
            // 未检测到目标状态：执行搜索模式（按时间规划搜索路径）
            gloalstaus = "没检测到";
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - no_search_start).count();

            // 搜索模式：（未检测到目标的时间， 起始记录的位置，板外控制插件, 当前相对高度 ）
            drone_controller.searchPattern(elapsed_time, start_position, offboard, state_machine.getNotDetectedAltitude());
        }
        else
        {
            // 待机状态：显示图像并等待指令
            gloalstaus = "待机中";
            logging::get_logger()->info("待机中");

            continue;
        }

        // 高度过低时跳过处理（防止碰撞）
        if (current_relative_altitude_m < 0.5)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 制循环频率
    }

    // 停止跟踪线程并释放资源
    tracker.stop();

    // 停止Offboard模式并发送降落指令
    offboard.stop();

    // 设置降落状态为true，通知状态机无人机已降落
    monitor.setHasLanded();
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 设置检测运行标志为false，结束检测线程
    detection_running = false;
}

/**
 * 启动着陆检测 并 发送控制命令
 *
 * 功能说明：
 * 1. 根据编译模式(真机/仿真)初始化摄像头
 * 2. 启动独立线程运行视觉检测算法
 * 3. 持续监控检测状态直到完成
 * 4. 确保资源正确释放
 */
void detectLandingPadAndSendCommand(Mavsdk_members &mavsdk)
{
    // 记录检测开始信息并设置状态标志
    logging::get_logger()->info("Starting detection ...");
    detection_running = true;

#ifdef REAL_HARDWARE

    // 真机环境下的摄像头初始化
    logging::get_logger()->info("在真机模式设置摄像头");
    cv::VideoCapture cap(0); // 打开默认摄像头(设备ID 0)

    // 检查摄像头是否成功打开
    if (!cap.isOpened())
    {
        logging::get_logger()->error("Error: Unable to open camera");
        return;
    }

    // 设置摄像头分辨率
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    // 启动检测线程(传递摄像头引用和类综合体)
    std::thread detector_thread(detection_thread, std::ref(mavsdk));

#elif defined(SIMULATION)

    // 仿真环境下的摄像头初始化
    logging::get_logger()->info("在仿真模式设置摄像头");

    // 启动Gazebo仿真相机模块
    GazeboCamera::Instance()->start();

    // 启动检测线程(传递摄像头引用和类综合体)
    std::thread detector_thread(detection_thread, std::ref(mavsdk));

#else
    // 未定义运行模式时的错误处理
    logging::get_logger()->error("mode error: 未定义运行模式(REAL_HARDWARE/SIMULATION)");
    return;

#endif

    // 主线程等待检测完成
    while (detection_running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // 间歇休眠减少CPU占用
    }

    // 确保检测线程正确结束
    if (detector_thread.joinable())
    {
        detector_thread.join(); // 等待线程结束并回收资源
    }

    logging::get_logger()->info("检测系统停止运行了");
}