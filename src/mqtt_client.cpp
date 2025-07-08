#include "mqtt_client.hpp"
#include "file_transfer.hpp"

namespace fs = std::filesystem; // 声明命名空间

/**
 * @brief MQTT客户端构造函数
 */
Mqtt::Mqtt() : client(BROKER, CLIENT_ID)
{
    // 设置回调
    client.set_callback(*this);
}

/**
 * @brief MQTT客户端析构函数
 */
Mqtt::~Mqtt()
{
    running = false;

    if (client.is_connected())
    {
        try
        {
            client.disconnect()->wait();
        }
        catch (const mqtt::exception &e)
        {
            std::cerr << "断开连接失败: " << e.what() << std::endl;
        }
    }
}

/**
 * @brief 初始化MQTT连接
 */
bool Mqtt::init()
{
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(KEEP_ALIVE);
    connOpts.set_user_name(USERNAME);
    connOpts.set_password(PASSWORD);
    connOpts.set_clean_session(true);

    try
    {
        client.connect(connOpts)->wait();
        sendMessage(REPLAY_TOPIC, "PX4 MQTT客户端已连接");
        return true;
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "连接失败: " << e.what() << std::endl;
        return false;
    }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/**
 * @brief 订阅主题并设置回调函数
 */
void Mqtt::subscribeTopic(const std::string &topic, MessageCallback callback)
{
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        topicCallbacks[topic] = callback;
    }

    try
    {
        client.subscribe(topic, 0)->wait();
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "订阅主题 " << topic << " 失败: " << e.what() << std::endl;
    }
}

/**
 * @brief 取消订阅主题
 */
void Mqtt::unsubscribeTopic(const std::string &topic)
{
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        topicCallbacks.erase(topic);
    }

    try
    {
        client.unsubscribe(topic)->wait();
    }
    catch (const mqtt::exception &e)
    {
        std::cerr << "取消订阅主题 " << topic << " 失败: " << e.what() << std::endl;
    }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/**
 * @brief 发送MQTT消息
 */
bool Mqtt::sendMessage(const std::string &topic, const std::string &payload)
{
    std::lock_guard<std::mutex> lock(sendMutex);
    if (!client.is_connected())
    {
        std::cerr << "未连接到代理，无法发送消息" << std::endl;
        return false;
    }

    try
    {
        mqtt::binary_ref binPayload(payload.data(), payload.size());
        auto msg = std::make_shared<mqtt::message>(topic, binPayload, 0, false);
        client.publish(msg)->wait();
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "发送消息失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief 连接丢失处理
 */
void Mqtt::connection_lost(const std::string &cause)
{
    std::cout << "连接丢失: " << cause << std::endl;
    std::cout << "尝试重新连接..." << std::endl;

    while (running && !client.is_connected())
    {
        try
        {
            client.reconnect()->wait();
            std::cout << "重新连接成功" << std::endl;
            sendMessage(REPLAY_TOPIC, "连接已恢复");
        }
        catch (const mqtt::exception &e)
        {
            std::cerr << "重连失败: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/**
 * @brief 消息到达处理
 */
void Mqtt::message_arrived(mqtt::const_message_ptr msg)
{
    std::string topic = msg->get_topic();
    std::vector<unsigned char> payload(msg->get_payload().begin(), msg->get_payload().end());

    // 处理文件传输消息
    if (topic.find("transferfiles/meta") != std::string::npos || topic.find("transferfiles/data/") != std::string::npos)
    {
        processFileTransferMessage(topic, payload); // 调用文件传输消息处理函数
        return;
    }

    // 处理普通消息
    std::cout << "\n收到消息 [主题: " << topic << "]: ";
    {
        std::lock_guard<std::mutex> lock(callbackMutex);
        auto it = topicCallbacks.find(topic);
        if (it != topicCallbacks.end())
        {
            it->second(payload);
        }
        else
        {
            std::cout << "内容长度 " << payload.size() << " 字节" << std::endl;
        }
    }
}
