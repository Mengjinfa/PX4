#include "mqtt_client.hpp"
#include <iostream>

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
 * @brief 获取当前系统状态
 */
SystemState Mqtt::getCurrentState() const
{
    return currentState.load();
}

/**
 * @brief 等待特定状态，带超时
 */
bool Mqtt::waitForState(SystemState targetState, std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(stateMutex);

    // 如果已经是目标状态，直接返回
    if (currentState == targetState)
    {
        return true;
    }

    // 等待状态改变
    return stateCv.wait_for(lock, timeout, [this, targetState]
                            { return currentState == targetState; });
}

/**
 * @brief 连接丢失处理
 */
void Mqtt::connection_lost(const std::string &cause)
{
    std::cout << "连接丢失: " << cause << std::endl;
    std::cout << "尝试重新连接..." << std::endl;

    // 更新状态为错误
    setSystemState(SystemState::ERROR);

    while (running && !client.is_connected())
    {
        try
        {
            client.reconnect()->wait();
            std::cout << "重新连接成功" << std::endl;
            sendMessage(REPLAY_TOPIC, "连接已恢复");

            // 恢复为空闲状态
            setSystemState(SystemState::IDLE);
        }
        catch (const mqtt::exception &e)
        {
            std::cerr << "重连失败: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}

/**
 * @brief 消息到达处理
 */
void Mqtt::message_arrived(mqtt::const_message_ptr msg)
{
    std::string topic = msg->get_topic();
    std::vector<unsigned char> payload(msg->get_payload().begin(), msg->get_payload().end());
    std::cout << "\n收到消息 [主题: " << topic << "]: ";

    // 查找并调用对应的回调函数
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

/**
 * @brief 交付完成回调
 */
void Mqtt::delivery_complete(mqtt::delivery_token_ptr token)
{
    // 可以实现消息交付确认的逻辑
}

/**
 * @brief 设置系统状态并通知等待线程
 */
void Mqtt::setSystemState(SystemState newState)
{
    {
        std::lock_guard<std::mutex> lock(stateMutex);
        currentState = newState;
    }
    stateCv.notify_all();
}