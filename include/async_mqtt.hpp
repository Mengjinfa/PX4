#ifndef ASYNC_MQTT_HPP
#define ASYNC_MQTT_HPP

#include "logger.hpp"
#include "mqtt/async_client.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/camera/camera.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/mission_raw/mission_raw.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <iostream>
#include <stdexcept>
#include <string>

extern std::atomic<int> start_detect;
extern std::atomic<int> find_landmark;
extern std::atomic<int> start_posctl;
extern std::atomic<int> is_take_off;

class Mavsdk_members
{
public:
    Mavsdk_members(mavsdk::MavlinkPassthrough &mp,
                   mavsdk::MissionRaw &mr,
                   mavsdk::Telemetry &t,
                   mavsdk::Offboard &o,
                   mavsdk::Mission &m,
                   mavsdk::Action &a,
                   mavsdk::Camera &c);

    mavsdk::MavlinkPassthrough &mavlink_passthrough;
    mavsdk::MissionRaw &mission_raw;
    mavsdk::Telemetry &telemetry;
    mavsdk::Offboard &offboard;
    mavsdk::Mission &mission;
    mavsdk::Action &action;
    mavsdk::Camera &camera;
};

class action_listener : public virtual mqtt::iaction_listener
{
public:
    // explicit action_listener(const std::string &name);
    // void on_failure(const mqtt::token &tok) override;
    // void on_success(const mqtt::token &tok) override;

private:
    std::string name_;
};

class callback : public virtual mqtt::callback, public virtual mqtt::iaction_listener
{
public:
    // callback(mqtt::async_client &cli, mqtt::connect_options &connOpts);

    // void connected(const std::string &cause) override;
    // void connection_lost(const std::string &cause) override;
    // void on_failure(const mqtt::token &tok) override;
    // void on_success(const mqtt::token &tok) override;
    // void message_arrived(mqtt::const_message_ptr msg) override;
    // void delivery_complete(mqtt::delivery_token_ptr token) override;

    // void set_reply_message(const std::string &msg);
    // void set_context(Mavsdk_members &mavsdk);

private:
    // void reconnect();

    // int nretry_;
    // mqtt::async_client &cli_;
    // mqtt::connect_options &connOpts_;
    // action_listener subListener_;
    // Mavsdk_members *context_;
    // std::string reply_msg_;
};

#endif