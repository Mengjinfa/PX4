#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>
#include <algorithm>
#include <atomic>
#include "target_tracker.hpp"
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <vector>
#include <apriltag/apriltag.h>
#include <apriltag/tag25h9.h>
#include "april_tag_detector.hpp"
#include "drone_controller.hpp"
#include "telemetry_monitor.hpp"
#include "video_display.hpp"
#include "detection_state_machine.hpp"
#include "sim_camera_module.h"
#include <mavsdk/mavsdk.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <fcntl.h>      // 提供 open 函数和 O_WRONLY、O_NOCTTY 等标志
#include <unistd.h>     // 提供 tcgetattr、tcsetattr、cfsetispeed、cfsetospeed 等函数
#include <termios.h>    // 提供 termios 结构体及相关配置常量（如 B115200、CLOCAL、CREAD 等）
#include <cerrno>       // 提供 errno 和错误处理相关定义

// 引入全局日志模块
#include "logger.hpp"

using namespace mavsdk;
using namespace cv;

std::atomic<bool> detection_running;

static int detection_count = 0;

std::queue<cv::Mat> image_queue;
std::mutex queue_mutex;
std::condition_variable queue_cond;

bool fifo_running = true;


void fifo_reader_thread() {
    const char* fifo_path = "/tmp/image_fifo";
    logging::get_logger()->info("在仿真模式准备打开接收图片帧管道");
    int fd = open(fifo_path, O_RDONLY | O_NONBLOCK);
    if (fd == -1) {
        logging::get_logger()->error("无法打开 FIFO");
        return;
    }

    char buffer[65536];

    while (fifo_running) {
        int bytes_read = read(fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            cv::Mat data(1, bytes_read, CV_8UC1, buffer);
            cv::Mat img = cv::imdecode(data, cv::IMREAD_COLOR);

            if (!img.empty()) {
                std::unique_lock<std::mutex> lock(queue_mutex);
                // 只保留最新的一帧，丢弃旧帧
                while (!image_queue.empty()) {
                    image_queue.pop();
                }
                image_queue.push(img.clone());
                lock.unlock();
                queue_cond.notify_one();
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    close(fd);
}

cv::Mat get_latest_frame() {
    // std::unique_lock<std::mutex> lock(queue_mutex);
    // queue_cond.wait(lock, []{ return !image_queue.empty(); });
    // cv::Mat frame = image_queue.front().clone();
    // image_queue.pop();
    cv::Mat frame = GazeboCamera::Instance()->GetNextFrame();
    return frame;
}

void detection_thread(cv::VideoCapture& cap, Offboard& offboard, Telemetry& telemetry, MavlinkPassthrough& mavlink_passthrough) {

    int fd; 
    fd = open("/dev/ttyUSB0", O_WRONLY | O_NOCTTY);

    if (fd == -1)
    {
        std::cerr << "无法打开串口设备" << std::endl;
        // return;
    }

// 配置串口
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








#ifdef REAL_HARDWARE
     logging::get_logger()->info("在真机模式");
#elif  defined(SIMULATION)
    logging::get_logger()->info("在仿真模式接收虚拟摄像头");
    // std::thread reader(fifo_reader_thread);
    // reader.detach();
    GazeboCamera::Instance()->start();
#else
    logging::get_logger()->error("mode error");
#endif


    AprilTagDetector detector;
    DroneController drone_controller(0.2f, 0.5f);
    TelemetryMonitor monitor(telemetry, 0.3f);
    std::this_thread::sleep_for(std::chrono::seconds(2));
    DetectionStateMachine state_machine;

    Offboard::VelocityBodyYawspeed zero_vel{};
    offboard.set_velocity_body(zero_vel);
    offboard.start();

    if (!offboard.is_active()) {
        logging::get_logger()->error("Offboard mode not active!");
        return;
    }

    std::chrono::seconds::rep elapsed;
    auto no_search_start = std::chrono::steady_clock::now();
    auto current_position = monitor.getCurrentPosition();
    auto start_position = current_position;
    float current_relative_altitude_m = monitor.getCurrentRelativeAltitudeM();

    logging::get_logger()->info("zxknorth_m: {}", current_position.north_m);
    logging::get_logger()->info("zxkeast_m: {}", current_position.east_m);
    logging::get_logger()->info("zxkdown_m: {}", current_position.down_m);
    logging::get_logger()->info("zxkcurrent_relative_altitude_m: {}", current_relative_altitude_m);
    double zxkx = 0;
    double zxky = 0;
    static auto lastWriteTime = std::chrono::steady_clock::now(); 
    while (detection_running && !monitor.hasLanded()) {


     auto zxkmode = telemetry.flight_mode();
     float zxkhight = current_relative_altitude_m;
     std::ostringstream oss;
     oss << std::fixed << std::setprecision(2);
     oss << "Telemetry Data: "
        << "error_x:" << zxkx << ", "
        << "error_y:" << zxky << ", "
        << "mode:" << zxkmode << ", "
        << "altitude:" << zxkhight << "m";
    
    // std::cout<<oss.str()<<std::endl;

    //进行判断距离上次超过1秒旧把oss.str()的数据 通过fd写到串口里
    if (std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::steady_clock::now() - lastWriteTime)
        .count() >= 1) 
    {
         std::cout<<oss.str()<<std::endl;
         std::string data = oss.str();
        ssize_t bytesWritten = write(fd, data.c_str(), data.size());
        if (bytesWritten == -1) {
            std::cerr << "写入串口失败" << std::endl;
        } 
        lastWriteTime = std::chrono::steady_clock::now();
    }


   




#ifdef REAL_HARDWARE
     logging::get_logger()->info("在真机模式获取视频帧");
    cv::Mat frame;
    cap >> frame;
#elif  defined(SIMULATION)
    logging::get_logger()->info("在仿真模式获取视频帧");
    cv::Mat frame = get_latest_frame(); 
    
#else
    logging::get_logger()->error("mode error");
#endif
        if (frame.empty()) {
            logging::get_logger()->warn("未接收到有效数据");
            Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = 0.0f;
            setpoint.east_m = 0.0f;
            setpoint.down_m = -6.0f;
            setpoint.yaw_deg = 0.0f;
            offboard.set_position_ned(setpoint);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        if (current_relative_altitude_m < 0.3) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        DetectionResult result = detector.detect(frame);
        current_position = monitor.getCurrentPosition();
        current_relative_altitude_m = monitor.getCurrentRelativeAltitudeM();
        state_machine.setCurrentRelativeAltitude(current_relative_altitude_m);
        DroneState state = state_machine.update(result.detected);

        if (state == DroneState::TRACKING && result.detected) {
            logging::get_logger()->info("检测到目标");
            auto center = result.centers[0];
            double dx = (center.x - result.width / 2.0) / result.width;
            double dy = (center.y - result.height / 2.0) / result.height;
            zxkx = dx ;
            zxky = dy;
            drone_controller.trackTarget(dx, dy, offboard, current_position, current_relative_altitude_m);

        } else if (state == DroneState::SEARCHING) {
            logging::get_logger()->info("正在查找");
            logging::get_logger()->info("查找高度值: {}", state_machine.getNotDetectedAltitude());

            Offboard::PositionNedYaw setpoint{};
            setpoint.north_m = current_position.north_m;
            setpoint.east_m = current_position.east_m;
            setpoint.down_m = -(state_machine.getNotDetectedAltitude());
            setpoint.yaw_deg = 0.0f;
            offboard.set_position_ned(setpoint);
            no_search_start = std::chrono::steady_clock::now();

        } else if (state == DroneState::NOT_DETECTED) {
            logging::get_logger()->info("没检测到");
            elapsed = std::chrono::duration_cast<std::chrono::seconds>(
                std::chrono::steady_clock::now() - no_search_start).count();
            drone_controller.searchPattern(elapsed, start_position, offboard, state_machine.getNotDetectedAltitude());

        } else {
            logging::get_logger()->info("待机中");
            VideoDisplay::showFrame(frame);
            if (VideoDisplay::shouldExit()) {
                // 可选退出逻辑
            }
            continue;
        }

        VideoDisplay::showFrame(frame);
        if (VideoDisplay::shouldExit()) {
            // 可选退出逻辑
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    offboard.stop();
    logging::get_logger()->info("准备退出");
    logging::get_logger()->info("monitor123的值: {}", monitor.hasLanded());

    monitor.setHasLanded();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    logging::get_logger()->info("monitor456的值: {}", monitor.hasLanded());

    detection_running = false;
    logging::get_logger()->info("已经退出11223");
    close(fd); // 关闭串口
}

void detectLandingPadAndSendCommand(Offboard& offboard, Telemetry& telemetry, MavlinkPassthrough& mavlink_passthrough) {
    logging::get_logger()->info("Starting detection ...");
    detection_running = true;

#ifdef REAL_HARDWARE
    logging::get_logger()->info("在真机模式设置摄像头");
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        logging::get_logger()->error("Error: Unable to open camera");
        return;
    }
     std::thread detector_thread(detection_thread, std::ref(cap), std::ref(offboard), std::ref(telemetry), std::ref(mavlink_passthrough));
#elif defined(SIMULATION)
    logging::get_logger()->info("在仿真模式设置摄像头");
    cv::VideoCapture cap(0);
    std::thread detector_thread(detection_thread, std::ref(cap), std::ref(offboard), std::ref(telemetry), std::ref(mavlink_passthrough));;

#else
     logging::get_logger()->error("mode error");
#endif

    while (detection_running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if (detector_thread.joinable()) {
        detector_thread.join();
    }

    logging::get_logger()->info("Detection system stopped");
}