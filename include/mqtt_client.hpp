#ifndef MQTT_CLIENT_HPP
#define MQTT_CLIENT_HPP

#include "mqtt/async_client.h" // MQTT客户端库
#include "singleton.hpp"

// 基础头文件
#include <atomic>
#include <cstddef>
#include <filesystem> // 文件系统库
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <nlohmann/json.hpp> // JSON库
#include <openssl/md5.h>     // OpenSSL库
#include <set>
#include <string>
#include <unordered_set>

// 命名空间声明（使用别名避免冲突）
using json = nlohmann::json; // JSON别名
namespace mqtt_ns = mqtt;    // MQTT库命名空间别名
namespace fs = std::filesystem;

// MQTT连接配置参数
const std::string BROKER = "223.94.45.64";     // MQTT代理服务器地址
const std::string USERNAME = "admin";          // MQTT用户名
const std::string PASSWORD = "senen!QAZxsw2";  // MQTT密码
const std::string REPLAY_TOPIC = "px4_replay"; // 回复主题
const std::string CLIENT_ID = "px4_receiver";  // MQTT客户端ID
const int PORT = 1883;                         // MQTT代理服务器端口
const int KEEP_ALIVE = 60;                     // 心跳间隔（秒）

/**
 * @brief MQTT客户端类
 * 提供MQTT通信功能，支持主题订阅、消息发布和状态管理
 */
class Mqtt : public virtual mqtt::callback // 继承mqtt::callback基类
{
private:
    mqtt::async_client client;       // MQTT客户端对象
    std::mutex sendMutex;            // 发送锁
    std::mutex callbackMutex;        // 回调锁
    std::atomic<bool> running{true}; // 运行标志

private:
    // 回调函数映射表
    using MessageCallback = std::function<void(const std::vector<unsigned char> &payload)>;
    std::map<std::string, MessageCallback> topicCallbacks;

    // 从mqtt::callback继承的接口实现
    void connection_lost(const std::string &cause) override;
    void message_arrived(mqtt::const_message_ptr msg) override;

public:
    Mqtt();
    ~Mqtt();

    bool init(); // 初始化MQTT连接

    void subscribeTopic(const std::string &topic, MessageCallback callback); // 订阅主题并设置回调函数
    void unsubscribeTopic(const std::string &topic);                         // 取消订阅主题

    bool sendMessage(const std::string &topic, const std::string &payload); // 发送MQTT消息
};

typedef NormalSingleton<Mqtt> mqtt_client;

#endif // MQTT_CLIENT_HPP
