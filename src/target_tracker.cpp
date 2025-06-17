#include "target_tracker.hpp"
#include "april_tag_detector.hpp"
#include "detection_state_machine.hpp"
#include "drone_controller.hpp"
#include "logger.hpp"
#include "sim_camera_module.h"
#include "telemetry_monitor.hpp"
#include "video_display.hpp"

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

std::atomic<bool> detection_running; // 检测线程运行标志（原子操作）
static int detection_count = 0;      // 检测计数（静态变量）
std::queue<cv::Mat> image_queue;     // 图像帧队列（用于线程间通信）
std::mutex queue_mutex;              // 队列互斥锁
std::condition_variable queue_cond;  // 条件变量（用于队列通知）
bool fifo_running = true;            // FIFO管道运行标志

/**
 * FIFO管道读取线程：从命名管道接收仿真图像数据
 */
void fifo_reader_thread()
{
    const char *fifo_path = "/tmp/image_fifo";
    logging::get_logger()->info("在仿真模式准备打开接收图片帧管道");

    // 以非阻塞模式打开FIFO管道
    int fd = open(fifo_path, O_RDONLY | O_NONBLOCK);

    if (fd == -1)
    {
        logging::get_logger()->error("无法打开 FIFO");
        return;
    }

    char buffer[65536]; // 缓冲区大小（64KB）

    while (fifo_running)
    {
        // 读取管道数据
        int bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read > 0)
        {
            // 将读取的字节流解码为OpenCV图像
            cv::Mat data(1, bytes_read, CV_8UC1, buffer);
            cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);

            if (!img.empty())
            {
                // 线程安全操作：只保留最新一帧，丢弃旧帧
                std::unique_lock<std::mutex> lock(queue_mutex);
                while (!image_queue.empty())
                {
                    image_queue.pop();
                }
                image_queue.push(img.clone());
                lock.unlock();
                queue_cond.notify_one(); // 通知等待的线程
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(fd); // 关闭管道
}

/**
 * 获取最新图像帧（仿真模式）
 * @return 最新的图像帧
 */
cv::Mat get_latest_frame()
{
    // 从Gazebo仿真相机获取图像
    cv::Mat frame = GazeboCamera::Instance()->GetNextFrame();
    return frame;
}

/**
 * 视觉检测与控制线程
 * @param cap 摄像头捕获对象（真机模式）
 * @param offboard 无人机Offboard控制插件
 * @param telemetry 无人机遥测数据插件
 * @param mavlink_passthrough MAVLink透传插件
 */
void detection_thread(cv::VideoCapture &cap, Offboard &offboard, Telemetry &telemetry, MavlinkPassthrough &mavlink_passthrough)
{
    // 打开串口设备（用于发送数据到地面站）
    int fd;
    fd = open("/dev/ttyUSB0", O_WRONLY | O_NOCTTY);

    if (fd == -1)
    {
        std::cerr << "无法打开串口设备" << std::endl;
        // return;  // 注意：此处未返回，即使串口打开失败仍继续执行
    }

    // 配置串口参数（波特率115200，8N1格式）
    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    tcsetattr(fd, TCSANOW, &options);

    // 根据编译模式选择运行环境
#ifdef REAL_HARDWARE
    logging::get_logger()->info("在真机模式");
#elif defined(SIMULATION)
    logging::get_logger()->info("在仿真模式接收虚拟摄像头");
    GazeboCamera::Instance()->start();
#else
    logging::get_logger()->error("mode error");
#endif

    // 初始化AprilTag检测器和控制器
    AprilTagDetector detector;
    DroneController drone_controller(0.2f, 0.5f);         // PID参数：水平0.2，垂直0.5
    TelemetryMonitor monitor(telemetry, 0.3f);            // 高度阈值0.3m
    std::this_thread::sleep_for(std::chrono::seconds(2)); // 等待系统初始化
    DetectionStateMachine state_machine;

    // 启动Offboard模式并设置初始速度为零
    Offboard::VelocityBodyYawspeed zero_vel{};
    offboard.set_velocity_body(zero_vel);
    offboard.start();

    if (!offboard.is_active())
    {
        logging::get_logger()->error("Offboard mode not active!");
        return;
    }

    // 记录初始位置和时间
    auto no_search_start = std::chrono::steady_clock::now();
    auto current_position = monitor.getCurrentPosition();
    auto start_position = current_position;
    float current_relative_altitude_m = monitor.getCurrentRelativeAltitudeM();

    // 记录初始位置信息（用于调试）
    logging::get_logger()->info("zxknorth_m: {}", current_position.north_m);
    logging::get_logger()->info("zxkeast_m: {}", current_position.east_m);
    logging::get_logger()->info("zxkdown_m: {}", current_position.down_m);
    logging::get_logger()->info("zxkcurrent_relative_altitude_m: {}", current_relative_altitude_m);

    double zxkx = 0;                                              // 目标水平偏差
    double zxky = 0;                                              // 目标垂直偏差
    static auto lastWriteTime = std::chrono::steady_clock::now(); // 上次串口写入时间

    // 主循环：持续检测和控制，直到停止标志或降落完成
    while (detection_running && !monitor.hasLanded())
    {
        // 获取当前飞行模式和高度
        auto zxkmode = telemetry.flight_mode();
        float zxkhight = current_relative_altitude_m;

        // 格式化遥测数据（用于串口发送）
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2);
        oss << "Telemetry Data: "
            << "error_x:" << zxkx << ", "
            << "error_y:" << zxky << ", "
            << "mode:" << zxkmode << ", "
            << "altitude:" << zxkhight << "m";

        // 每秒向串口发送一次数据
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - lastWriteTime).count() >= 1)
        {
            std::cout << oss.str() << std::endl;
            std::string data = oss.str();
            ssize_t bytesWritten = write(fd, data.c_str(), data.size());
            if (bytesWritten == -1)
            {
                std::cerr << "写入串口失败" << std::endl;
            }
            lastWriteTime = std::chrono::steady_clock::now();
        }

        // 根据编译模式获取图像帧
#ifdef REAL_HARDWARE
        logging::get_logger()->info("在真机模式获取视频帧");
        cv::Mat frame;
        cap >> frame;
#elif defined(SIMULATION)
        logging::get_logger()->info("在仿真模式获取视频帧");
        cv::Mat frame = get_latest_frame();
#else
        logging::get_logger()->error("mode error");
#endif

        // 处理无效帧：保持当前位置，继续循环
        if (frame.empty())
        {
            logging::get_logger()->warn("未接收到有效数据");
            Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = 0.0f;
            setpoint.east_m = 0.0f;
            setpoint.down_m = -6.0f; // 保持6米高度
            setpoint.yaw_deg = 0.0f;
            offboard.set_position_ned(setpoint);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // 低空保护：高度低于0.3m时不执行检测
        if (current_relative_altitude_m < 0.3)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        // 执行AprilTag检测
        DetectionResult result = detector.detect(frame, true); // 绘制检测结果
        current_position = monitor.getCurrentPosition();
        current_relative_altitude_m = monitor.getCurrentRelativeAltitudeM();
        state_machine.setCurrentRelativeAltitude(current_relative_altitude_m);

        // 更新状态机（根据是否检测到目标）
        DroneState state = state_machine.update(result.detected);

        // 状态处理：跟踪目标
        if (state == DroneState::TRACKING && result.detected)
        {
            logging::get_logger()->info("检测到目标");
            auto center = result.centers[0];

            // 计算目标在图像中的相对偏移（归一化到[-0.5, 0.5]）
            double dx = (center.x - result.width / 2.0) / result.width;
            double dy = (center.y - result.height / 2.0) / result.height;
            zxkx = dx;
            zxky = dy;
            // 控制无人机跟踪目标
            drone_controller.trackTarget(dx, dy, offboard, current_position, current_relative_altitude_m);
        }
        // 状态处理：搜索目标（保持当前高度）
        else if (state == DroneState::SEARCHING)
        {
            logging::get_logger()->info("正在查找");
            logging::get_logger()->info("查找高度值: {}", state_machine.getNotDetectedAltitude());

            // 保持当前位置和高度
            Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = current_position.north_m;
            setpoint.east_m = current_position.east_m;
            setpoint.down_m = -(state_machine.getNotDetectedAltitude());
            setpoint.yaw_deg = 0.0f;
            offboard.set_position_ned(setpoint);
            no_search_start = std::chrono::steady_clock::now(); // 重置未检测时间
        }
        // 状态处理：长时间未检测到目标（执行搜索模式）
        else if (state == DroneState::NOT_DETECTED)
        {
            logging::get_logger()->info("没检测到");

            // 计算未检测时间
            elapsed = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - no_search_start).count();

            // 执行搜索模式（如螺旋搜索）
            drone_controller.searchPattern(elapsed, start_position, offboard, state_machine.getNotDetectedAltitude());
        }
        // 状态处理：待机
        else
        {
            logging::get_logger()->info("待机中");
            VideoDisplay::showFrame(frame); // 显示图像
            continue;
        }

        // 显示图像并检查退出条件
        VideoDisplay::showFrame(frame);

        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // 控制循环频率（约50Hz）
    }

    // 退出处理
    offboard.stop(); // 停止Offboard模式
    logging::get_logger()->info("准备退出");
    logging::get_logger()->info("monitor123的值: {}", monitor.hasLanded());

    monitor.setHasLanded(); // 设置降落状态
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    logging::get_logger()->info("monitor456的值: {}", monitor.hasLanded());

    detection_running = false;
    logging::get_logger()->info("已经退出11223");
    close(fd); // 关闭串口
}

/**
 * 启动视觉检测与控制
 * @param offboard 无人机Offboard控制插件
 * @param telemetry 无人机遥测数据插件
 * @param mavlink_passthrough MAVLink透传插件
 */
void detectLandingPadAndSendCommand(Offboard &offboard, Telemetry &telemetry, MavlinkPassthrough &mavlink_passthrough)
{
    logging::get_logger()->info("Starting detection ...");
    detection_running = true;

    // 根据编译模式初始化摄像头并启动检测线程
#ifdef REAL_HARDWARE
    logging::get_logger()->info("在真机模式设置摄像头");
    cv::VideoCapture cap(0);
    if (!cap.isOpened())
    {
        logging::get_logger()->error("Error: Unable to open camera");
        return;
    }
    std::thread detector_thread(detection_thread, std::ref(cap), std::ref(offboard), std::ref(telemetry), std::ref(mavlink_passthrough));
#elif defined(SIMULATION)
    logging::get_logger()->info("在仿真模式设置摄像头");
    cv::VideoCapture cap(0);
    std::thread detector_thread(detection_thread, std::ref(cap), std::ref(offboard), std::ref(telemetry), std::ref(mavlink_passthrough));
#else
    logging::get_logger()->error("mode error");
#endif

    // 等待检测线程结束
    while (detection_running)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 确保线程正确加入
    if (detector_thread.joinable())
    {
        detector_thread.join();
    }

    logging::get_logger()->info("Detection system stopped");
}