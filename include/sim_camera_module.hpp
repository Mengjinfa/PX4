#pragma once
#include "singleton.hpp"
#include <atomic>
#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <queue>
#include <thread>

class CGazebo_camera
{
public:
    explicit CGazebo_camera();
    ~CGazebo_camera();

    void init(int argc, char **argv, const std::string &topic);
    void start();
    void stop();
    cv::Mat GetNextFrame();

private:
    void ImageCallback(const boost::shared_ptr<const gazebo::msgs::ImageStamped> &_msg);
    void DisplayThread();

    bool stopped_; // 停止标志

    std::queue<cv::Mat> frame_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cond_;
    std::atomic<bool> running_{false};
    std::thread display_thread_;
    std::string topic_;

    gazebo::transport::NodePtr node_;
    gazebo::transport::SubscriberPtr sub_;
};

typedef NormalSingleton<CGazebo_camera> GazeboCamera;
