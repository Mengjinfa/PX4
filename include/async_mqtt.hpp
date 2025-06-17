#ifndef SUBSCRIBE_HPP
#define SUBSCRIBE_HPP

#include "mqtt/async_client.h"
#include <iostream>
#include <stdexcept>
#include <string>

// 默认服务器URI和客户端ID
const std::string DFLT_SERVER_URI("mqtt://47.123.5.22:1883");
// const std::string DFLT_SERVER_URI("mqtt://192.168.1.169:1883");
const std::string CLIENT_ID("paho_cpp_async_subscribesim");
const std::string MQTTUSER("admin");
const std::string MQTTPASSWORD("public");
const std::string TOPICCMD("commandsender111");
const std::string TOPICBEIDOU1("testup");
const std::string TOPICBEIDOU2("testup1");

const int QOS = 2;
const int N_RETRY_ATTEMPTS = 5;

// 操作监听器类
class action_listener : public virtual mqtt::iaction_listener
{
public:
    explicit action_listener(const std::string &name);
    void on_failure(const mqtt::token &tok) override;
    void on_success(const mqtt::token &tok) override;

private:
    std::string name_;
};

// 主回调函数类
class callback : public virtual mqtt::callback, public virtual mqtt::iaction_listener
{
public:
    callback(mqtt::async_client &cli, mqtt::connect_options &connOpts);

    // 连接相关回调
    void connected(const std::string &cause) override;
    void connection_lost(const std::string &cause) override;
    void on_failure(const mqtt::token &tok) override;
    void on_success(const mqtt::token &tok) override;

    // 消息到达回调
    void message_arrived(mqtt::const_message_ptr msg) override;
    void delivery_complete(mqtt::delivery_token_ptr token) override;

private:
    void reconnect();

    int nretry_;
    mqtt::async_client &cli_;
    mqtt::connect_options &connOpts_;
    action_listener subListener_;
};

// 函数声明（需要在实际.cpp文件中实现）
void process_command(const std::string &json);
void parse_json_message(const std::string &json_string);

#endif
