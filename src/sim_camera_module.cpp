#include "sim_camera_module.hpp"
#include "mqtt_client.hpp"

using namespace gazebo;

// CGazebo_camera 类的定义
// 构造函数初始化 running_ 和 stopped_ 标志，分别表示相机是否正在运行和是否已停止。
CGazebo_camera::CGazebo_camera() : running_(false), stopped_(false) {}

// 析构函数调用 stop() 方法，确保在对象销毁时停止相机的运行。
CGazebo_camera::~CGazebo_camera()
{
    stop();
}

// 初始化相机模块
void CGazebo_camera::init(int argc, char **argv, const std::string &topic)
{
    topic_ = topic;                                                    // 设置相机图像的主题名称
    gazebo::client::setup(argc, argv);                                 // 初始化 Gazebo 客户端
    node_ = gazebo::transport::NodePtr(new gazebo::transport::Node()); // 创建一个 Gazebo 传输节点
    node_->Init();                                                     // 初始化节点
}

// 启动相机的运行
void CGazebo_camera::start()
{
    if (!running_)
    {
        running_ = true;                                                                                   // 设置运行标志为 true
        sub_ = node_->Subscribe<gazebo::msgs::ImageStamped>(topic_, &CGazebo_camera::ImageCallback, this); // 订阅 Gazebo 的图像主题，并将回调函数设置为 ImageCallback
    }
}

// 停止相机的运行
void CGazebo_camera::stop()
{
    if (running_)
    {
        running_ = false; // 设置运行标志为 false
        if (display_thread_.joinable())
        {
            display_thread_.join(); // 等待显示线程完成
        }

        std::lock_guard<std::mutex> lock(queue_mutex_); // 使用互斥锁保护队列
        stopped_ = true;                                // 设置停止标志为 true

        queue_cond_.notify_all();   // 唤醒所有等待线程
        gazebo::client::shutdown(); // 关闭 Gazebo 客户端
    }
}

// 获取下一帧图像
cv::Mat CGazebo_camera::GetNextFrame()
{
    std::unique_lock<std::mutex> lock(queue_mutex_); // 使用互斥锁保护队列

    // 等待直到队列非空或停止标志被设置
    queue_cond_.wait(lock, [this]
                     { return stopped_ || !frame_queue_.empty(); });

    if (stopped_)
    {
        return cv::Mat(); // 如果已停止，返回空帧
    }

    cv::Mat frame = frame_queue_.front(); // 获取队列中的第一帧
    frame_queue_.pop();                   // 从队列中移除该帧

    return frame; // 返回获取的帧
}

// 图像回调函数
void CGazebo_camera::ImageCallback(const boost::shared_ptr<const gazebo::msgs::ImageStamped> &_msg)
{
    const auto &img_msg = _msg->image(); // 获取图像消息
    const int width = img_msg.width();   // 获取图像宽度
    const int height = img_msg.height(); // 获取图像高度
    const int step = img_msg.step();     // 获取图像步长

    cv::Mat frame(height, width, CV_8UC3, const_cast<uchar *>(reinterpret_cast<const uchar *>(img_msg.data().c_str())), step); // 将 Gazebo 图像数据转换为 OpenCV 的 cv::Mat 格式
    cv::Mat bgr_frame;                                                                                                         // 定义 BGR 格式的图像
    cv::cvtColor(frame, bgr_frame, cv::COLOR_RGB2BGR);                                                                         // 将 RGB 图像转换为 BGR 格式

    std::lock_guard<std::mutex> lock(queue_mutex_); // 使用互斥锁保护队列
    if (frame_queue_.size() >= 2)
    {
        frame_queue_.pop(); // 如果队列中有超过 2 帧，移除最早的一帧
    }

    frame_queue_.push(bgr_frame.clone()); // 将转换后的图像添加到队列中
    queue_cond_.notify_one();             // 通知等待的消费者线程
}

// 显示线程函数
void CGazebo_camera::DisplayThread()
{
    cv::namedWindow("Gazebo Camera", cv::WINDOW_NORMAL);            // 创建一个 OpenCV 窗口
    cv::resizeWindow("Gazebo Camera", 800, 600);                    // 设置窗口大小为 800x600
    cv::setWindowProperty("Gazebo Camera", cv::WND_PROP_OPENGL, 1); // 设置窗口属性为 OpenGL

    constexpr int target_fps = 30;                                            // 目标帧率为 30 FPS
    const auto frame_duration = std::chrono::milliseconds(1000 / target_fps); // 每帧的持续时间

    while (running_)
    {
        auto start = std::chrono::steady_clock::now(); // 获取当前时间
        cv::Mat display_frame;                         // 定义用于显示的图像
        {
            std::lock_guard<std::mutex> lock(queue_mutex_); // 使用互斥锁保护队列
            if (!frame_queue_.empty())
            {
                display_frame = frame_queue_.front(); // 获取队列中的第一帧
                frame_queue_.pop();                   // 从队列中移除该帧
            }
        }

        auto elapsed = std::chrono::steady_clock::now() - start; // 计算已过去的时间
        auto sleep_time = frame_duration - elapsed;              // 计算剩余时间
        if (sleep_time > std::chrono::milliseconds(0))
        {
            std::this_thread::sleep_for(sleep_time); // 如果还有剩余时间，线程休眠
        }
    }

    cv::destroyAllWindows(); // 关闭所有 OpenCV 窗口
}