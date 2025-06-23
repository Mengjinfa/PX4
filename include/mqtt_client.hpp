#ifndef MQTT_CLIENT_HPP
#define MQTT_CLIENT_HPP

// 基础头文件
#include "mqtt/async_client.h" // MQTT客户端库
#include <cstddef>             // size_t
#include <filesystem>          // C++17文件系统库，用于目录创建和文件操作
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>
#include <set>
#include <string>
#include <vector>

// 命名空间声明（使用别名避免冲突）
using json = nlohmann::json; // JSON别名
namespace mqtt_ns = mqtt;    // MQTT库命名空间别名

// MQTT连接配置参数
const std::string BROKER = "223.94.45.64";         // MQTT代理服务器地址
const int PORT = 1883;                             // MQTT代理服务器端口
const std::string USERNAME = "admin";              // MQTT用户名
const std::string PASSWORD = "senen!QAZxsw2";      // MQTT密码
const std::string TOPIC = "transferfiles/#";       // 订阅的主题（接收所有文件传输相关消息）
const std::string REPLAY_TOPIC = "px4_replay";     // 回复主题
const std::string ACK_TOPIC = "transferfiles/ack"; // 确认消息发送主题
const std::string CLIENT_ID = "px4_receiver";      // MQTT客户端ID
const int KEEP_ALIVE = 60;                         // 心跳间隔（秒）

/**
 * @class Mqtt
 * @brief 基于MQTT的文件分块接收系统
 * 支持大文件分块传输、断点续传、MD5校验和自动重连
 * 使用C++17特性和现代设计模式
 *
 * 定义了一个名为Mqtt的类，它采用虚继承的方式从mqtt_ns::callback类派生而来。
 * public继承：这意味着Mqtt类会公开继承mqtt_ns::callback类的接口。也就是说，Mqtt对象可以被当作mqtt_ns::callback对象来使用，比如用于参数传递。
 * 命名空间限定：mqtt_ns::callback表明callback类位于mqtt_ns命名空间中，这样做是为了避免命名冲突。
 */
class Mqtt : public virtual mqtt_ns::callback // 使用命名空间别名
{
private:
    // 文件存储配置
    std::string saveDir; // 文件保存目录

    // MQTT客户端实例
    mqtt::async_client client; // 异步MQTT客户端

    std::mutex fileMutex;                   // 保护文件操作的互斥锁
    std::mutex topicMutex;                  // 保护主题集合的互斥锁
    std::mutex sendMutex;                   // 消息发送互斥锁
    std::set<std::string> subscribedTopics; // 存储已订阅的主题集合

    // 文件信息结构体 - 存储当前接收文件的元数据和内容
    struct FileInfo
    {
        std::string name;                             // 文件名
        size_t size;                                  // 文件总大小（字节）
        size_t chunks;                                // 文件总分块数
        size_t received;                              // 已接收块数
        std::vector<std::vector<unsigned char>> data; // 块数据存储（索引为块ID）
        bool isLast;                                  // 是否为最后一个文件
        std::string md5;                              // 文件MD5校验和
    };
    std::unique_ptr<FileInfo> currentFile; // 当前接收文件的智能指针

public:
    explicit Mqtt(const std::string &saveDir); // 构造函数 - 指定文件保存目录

    // MQTT函数
    bool init(); // 初始化MQTT连接

    // 订阅主题
    void Monitor_flight_file();                                   // 监听航线文件MQTT消息
    void subscribeTopic(const std::string &topic);                // 订阅单个主题
    void subscribeTopics(const std::vector<std::string> &topics); // 订阅多个主题

    // 消息发送功能
    bool sendMessage(const std::string &topic, const std::string &payload);
    size_t sendMessages(const std::vector<std::pair<std::string, std::string>> &messages);

private:
    void createDirectory(const std::string &path);                                  // 创建目录(如果不存在)
    std::string calculateMD5(const std::string &filePath);                          // 计算文件的MD5校验和
    void handleMetaMessage(const std::string &payload);                             // 处理元数据消息(文件信息)
    void handleDataMessage(int chunkId, const std::vector<unsigned char> &payload); // 处理数据块消息
    void saveCurrentFile();                                                         // 保存完整文件到磁盘
    void sendAck(const std::string &fileName, const std::string &status);           // 发送确认消息
    void connection_lost(const std::string &cause) override;                        // MQTT连接丢失
    void message_arrived(mqtt_ns::const_message_ptr msg) override;                  // MQTT消息到达
    void handle_Test_Message(const std::vector<unsigned char> &payload);            // 接收到话题后的处理
};

#endif // MQTT_CLIENT_HPP