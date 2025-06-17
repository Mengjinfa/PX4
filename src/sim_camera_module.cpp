#include "sim_camera_module.h"

using namespace gazebo;

CGazebo_camera::CGazebo_camera(): 
running_(false),
stopped_(false) {}

CGazebo_camera::~CGazebo_camera() {
    stop();
}

void CGazebo_camera::init(int argc, char** argv,const std::string& topic) {
    topic_ =  topic;
    gazebo::client::setup(argc, argv);
    node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    node_->Init();
}

void CGazebo_camera::start() {
    if (!running_) {
        running_ = true;
        sub_ = node_->Subscribe<gazebo::msgs::ImageStamped>(topic_,&CGazebo_camera::ImageCallback, this);
        // display_thread_ = std::thread(&CGazebo_camera::DisplayThread, this);
    }
}

void CGazebo_camera::stop() {
    if (running_) {
        running_ = false;
        if (display_thread_.joinable()) {
            display_thread_.join();
        }
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            stopped_ = true;
        }
        queue_cond_.notify_all(); // 唤醒所有等待线程
        gazebo::client::shutdown();
    }
}

cv::Mat CGazebo_camera::GetNextFrame() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    
    // 等待直到队列非空或停止标志被设置
    queue_cond_.wait(lock, [this] {
        return stopped_ || !frame_queue_.empty();
    });

    if (stopped_) {
        return cv::Mat(); // 返回空帧表示停止
    }

    cv::Mat frame = frame_queue_.front();
    frame_queue_.pop();
    return frame;
}

void CGazebo_camera::ImageCallback(const boost::shared_ptr<const gazebo::msgs::ImageStamped> &_msg) {
    const auto& img_msg = _msg->image();
    const int width = img_msg.width();
    const int height = img_msg.height();
    const int step = img_msg.step();

    cv::Mat frame(height, width, CV_8UC3, const_cast<uchar*>(reinterpret_cast<const uchar*>(img_msg.data().c_str())), step);

    cv::Mat bgr_frame;
    cv::cvtColor(frame, bgr_frame, cv::COLOR_RGB2BGR);

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (frame_queue_.size() >= 2) {
            frame_queue_.pop();
        }
        frame_queue_.push(bgr_frame.clone());
        queue_cond_.notify_one();  // 通知等待的消费者线程
    }
}

void CGazebo_camera::DisplayThread() {
    cv::namedWindow("Gazebo Camera", cv::WINDOW_NORMAL);
    cv::resizeWindow("Gazebo Camera", 800, 600); // 设置初始大小为800x600
    cv::setWindowProperty("Gazebo Camera", cv::WND_PROP_OPENGL, 1);

    constexpr int target_fps = 30;
    const auto frame_duration = std::chrono::milliseconds(1000 / target_fps);

    while (running_) {
        auto start = std::chrono::steady_clock::now();
        cv::Mat display_frame;
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (!frame_queue_.empty()) {
                display_frame = frame_queue_.front();
                frame_queue_.pop();
            }
        }

        if (!display_frame.empty()) {
            cv::imshow("Gazebo Camera", display_frame);
            cv::waitKey(1);
        }

        auto elapsed = std::chrono::steady_clock::now() - start;
        auto sleep_time = frame_duration - elapsed;
        if (sleep_time > std::chrono::milliseconds(0)) {
            std::this_thread::sleep_for(sleep_time);
        }
    }
    cv::destroyAllWindows();
}

