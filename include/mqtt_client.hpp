#ifndef MQTT_CLIENT_HPP
#define MQTT_CLIENT_HPP

#include "mqtt/async_client.h" // MQTT客户端库

// 基础头文件
#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <unordered_set>

// 命名空间声明（使用别名避免冲突）
using json = nlohmann::json; // JSON别名
namespace mqtt_ns = mqtt;    // MQTT库命名空间别名

// MQTT连接配置参数
const std::string BROKER = "223.94.45.64";     // MQTT代理服务器地址
const int PORT = 1883;                         // MQTT代理服务器端口
const std::string USERNAME = "admin";          // MQTT用户名
const std::string PASSWORD = "senen!QAZxsw2";  // MQTT密码
const std::string REPLAY_TOPIC = "px4_replay"; // 回复主题
const std::string CLIENT_ID = "px4_receiver";  // MQTT客户端ID
const int KEEP_ALIVE = 60;                     // 心跳间隔（秒）

// 定义系统状态枚举
enum class SystemState
{
    IDLE,             // 空闲状态
    STARTING_MACHINE, // 启动状态机
    PROCESSING,       // 处理中
    ERROR             // 错误状态
};

/**
 * @brief MQTT客户端类
 * 提供MQTT通信功能，支持主题订阅、消息发布和状态管理
 */
class Mqtt : public virtual mqtt::callback
{
private:
    mqtt::async_client client;
    std::mutex sendMutex;
    std::mutex callbackMutex;
    std::atomic<bool> running{true};

    // 状态管理
    std::atomic<SystemState> currentState{SystemState::IDLE};
    std::condition_variable stateCv;
    std::mutex stateMutex;

    // 回调函数映射表
    using MessageCallback = std::function<void(const std::vector<unsigned char> &payload)>;
    std::map<std::string, MessageCallback> topicCallbacks;

public:
    Mqtt();
    ~Mqtt();

    // 禁止拷贝构造和赋值
    Mqtt(const Mqtt &) = delete;
    Mqtt &operator=(const Mqtt &) = delete;

    bool init(); // 初始化MQTT连接

    void subscribeTopic(const std::string &topic, MessageCallback callback); // 订阅主题并设置回调函数
    void unsubscribeTopic(const std::string &topic);                         // 取消订阅主题

    bool sendMessage(const std::string &topic, const std::string &payload);                                           // 发送MQTT消息
    SystemState getCurrentState() const;                                                                              // 获取当前系统状态
    bool waitForState(SystemState targetState, std::chrono::milliseconds timeout = std::chrono::milliseconds::max()); // 等待特定状态，带超时

    // 从mqtt::callback继承的接口实现
    void connection_lost(const std::string &cause) override;
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;

private:
    void setSystemState(SystemState newState); // 设置系统状态并通知等待线程
};

#endif // MQTT_CLIENT_HPP
