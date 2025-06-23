#include "async_mqtt.hpp"
#include "flight_procedure.hpp"
#include "mqtt_client.hpp"
#include "sim_camera_module.hpp"
#include "takeoff_and_land.hpp"
#include "target_tracker.hpp"
#include "vision_guided_mission_flying.hpp"

#include <chrono> // 用于延时和超时
#include <fcntl.h>
#include <fstream>
#include <mutex>
#include <nlohmann/json.hpp>
#include <thread>
#include <unistd.h>
using namespace mavsdk;

// 全局原子标志（用于多线程同步）
// 原子操作确保标志修改在多线程环境下的线程安全性
std::atomic<int> start_detect(0);  // 0:未开始检测 1:已启动目标检测
std::atomic<int> find_landmark(0); // 0:未发现地标 1:已识别地标
std::atomic<int> start_posctl(0);  // 0:未启动位置控制 1:已激活位置控制模式
std::atomic<int> is_take_off(0);   // 0:未起飞 1:已起飞

// // Gazebo仿真环境中相机图像主题（用于订阅视觉数据）
// std::string subscribePtr = "/gazebo/default/iris/base_link/camera/image";

// Mavsdk_members类的构造函数（初始化对象）
// 功能：将各MAVSDK插件实例绑定到Mavsdk_members
Mavsdk_members::Mavsdk_members(MavlinkPassthrough &mp, MissionRaw &mr, Telemetry &t, Offboard &o, Mission &m, Action &a, Camera &c)
    : action(a), mission(m), telemetry(t), offboard(o), camera(c), mavlink_passthrough(mp), mission_raw(mr) {}

// // MQTT动作监听器类（处理MQTT操作的回调）
// action_listener::action_listener(const std::string &name) : name_(name) {}

// // MQTT操作失败回调
// void action_listener::on_failure(const mqtt::token &tok)
// {
//     // 记录失败日志，包含操作名称便于定位问题
//     logging::get_logger()->error("Action listener failure for {}", name_);
// }

// // MQTT操作成功回调（当前为空实现，可根据需求扩展）
// void action_listener::on_success(const mqtt::token &tok) {}

// /**
//  * @beilf:callback类构造函数：初始化MQTT回调处理器
//  * @param cli - MQTT异步客户端引用，用于消息发布与连接操作
//  * @param connOpts - MQTT连接选项引用，包含服务器认证等配置
//  */
// callback::callback(mqtt::async_client &cli, mqtt::connect_options &connOpts)
//     : cli_(cli),                                   // 保存客户端引用以便后续操作
//       connOpts_(connOpts),                         // 保存连接选项以便重连时复用
//       subListener_("Subscription"),                // 初始化订阅监听器（处理订阅状态回调）
//       context_(nullptr),                           // 无人机上下文指针初始化为空，需通过set_context设置
//       reply_msg_("机载电脑mqtt已经连接上了服务器") // 连接成功默认回复消息
// {
// }

// // 设置无人机上下文对象
// // 作用：将DroneContext实例与回调处理器关联，使回调中可访问无人机控制接口
// void callback::set_context(Mavsdk_members &mavsdk)
// {
//     context_ = &mavsdk; // 存储上下文对象的地址
// }

// // MQTT重连机制实现
// // 功能：当连接断开时自动尝试重新连接到服务器
// void callback::reconnect()
// {
//     // 断开连接后等待 2秒 再尝试重连（避免频繁重连）
//     std::this_thread::sleep_for(std::chrono::milliseconds(2000));

//     try
//     {
//         // 使用保存的连接选项重新连接服务器，并绑定当前回调处理器
//         cli_.connect(connOpts_, nullptr, *this);
//     }
//     catch (const mqtt::exception &exc)
//     {
//         // 捕获连接异常并记录错误信息
//         logging::get_logger()->error("Error: {}", exc.what());

//         // 重连次数计数器
//         if (++nretry_ > N_RETRY_ATTEMPTS)
//         {
//             // 达到最大重连次数时输出严重错误并退出程序
//             logging::get_logger()->critical("Max retry attempts reached. Exiting...");
//             exit(1);
//         }

//         // 递归调用重连方法，继续尝试连接
//         reconnect();
//     }
// }

// // MQTT操作失败回调处理
// // 触发场景：连接失败、发布/订阅操作失败时调用
// void callback::on_failure(const mqtt::token &tok)
// {
//     // 记录操作失败日志
//     logging::get_logger()->error("Connection attempt failed");

//     // 重连次数计数器递增
//     if (++nretry_ > N_RETRY_ATTEMPTS)
//     {
//         // 达到最大重连次数时输出严重错误并退出程序
//         logging::get_logger()->critical("Max retry attempts reached. Exiting...");
//         exit(1);
//     }

//     // 调用重连方法尝试重新连接
//     reconnect();
// }

// /**
//  * @beilf:设置MQTT连接成功后的回复消息
//  * @param msg - 待设置的回复消息内容
//  */
// void callback::set_reply_message(const std::string &msg)
// {
//     reply_msg_ = msg; // 更新回复消息内容
// }

// // MQTT操作成功回调（空实现，可根据需求扩展）
// void callback::on_success(const mqtt::token &tok) {}

// // MQTT连接成功回调
// // 触发场景：与MQTT服务器建立连接后调用
// void callback::connected(const std::string &cause)
// {
//     // 记录连接成功日志
//     logging::get_logger()->info("\nConnection success");

//     // 输出订阅信息（主题、客户端ID、QoS等级）
//     logging::get_logger()->info("\nSubscribing to topic '{}' for client {} using QoS{}\n", TOPICCMD, CLIENT_ID, QOS);
//     logging::get_logger()->info("Press Q<Enter> to quit\n");

//     // 订阅命令主题（TOPICCMD），使用指定QoS等级，并绑定订阅监听器
//     cli_.subscribe(TOPICCMD, QOS, nullptr, subListener_);

//     // 输出调试信息（示例字符串"zxk11111111"）
//     std::cout << "zxk11111111" << std::endl;

//     // 创建并发布连接成功回复消息
//     auto pubmsg = mqtt::make_message(PX4_REPLY, reply_msg_); // 创建消息对象
//     pubmsg->set_qos(QOS);                                    // 设置消息QoS等级
//     auto token = cli_.publish(pubmsg);                       // 发布消息
//     logging::get_logger()->info("Sent message to topic PX4_REPLY: {}\n", reply_msg_);
// }

// // MQTT连接丢失回调
// // 触发场景：网络断开或服务器主动断开连接时调用
// void callback::connection_lost(const std::string &cause)
// {
//     // 记录连接丢失警告日志
//     logging::get_logger()->warn("\nConnection lost");
//     if (!cause.empty())
//     {
//         logging::get_logger()->warn("\tcause: {}", cause); // 输出断开原因（如网络故障）
//     }

//     // 输出重连提示信息
//     logging::get_logger()->info("Reconnecting...");
//     nretry_ = 0; // 重置重连次数计数器
//     reconnect(); // 调用重连方法尝试重新连接
// }

// // 命令处理函数声明（处理MQTT接收到的JSON命令）
// void process_command(const std::string &json, Mavsdk_members &mavsdk);

// /**
//  * @beilf:JSON消息解析函数（封装命令处理逻辑）
//  * @param json_string - 接收到的JSON格式消息
//  * @param context - 无人机上下文对象（包含各控制插件）
//  */
// void parse_json_message(const std::string &json_string, Mavsdk_members &mavsdk)
// {
//     process_command(json_string, mavsdk); // 调用命令处理函数
// }

// // MQTT消息到达回调
// // 触发场景：接收到服务器发布的消息时调用
// void callback::message_arrived(mqtt::const_message_ptr msg)
// {
//     // 记录消息到达日志（包含主题和 payload 内容）
//     logging::get_logger()->info("Message arrived");
//     logging::get_logger()->info("\ttopic: '{}'", msg->get_topic());
//     logging::get_logger()->info("\tpayload: '{}'", msg->to_string());

//     // 提取消息内容
//     std::string json_str = msg->to_string();

//     // 检查消息主题是否为命令主题（TOPICCMD）
//     if (TOPICCMD == msg->get_topic())
//     {
//         // 检查上下文是否已初始化
//         if (context_)
//         {
//             // 创建新线程处理命令（避免阻塞MQTT接收线程）
//             std::thread t(
//                 [json_str, this]()
//                 {
//                     parse_json_message(json_str, *context_); // 解析并处理JSON命令
//                     logging::get_logger()->info("该指令运行结束");
//                 });
//             t.detach(); // 分离线程（线程自主运行，无需主线程等待）
//         }
//         else
//         {
//             // 上下文未初始化时记录错误
//             logging::get_logger()->error("Context not initialized in message handler");
//         }
//         logging::get_logger()->info("该指令运行结束222222222222222222222");
//     }
// }

// /**
//  * @beilf:MQTT消息投递完成回调函数  当消息成功发送到服务器或确认丢失时触发
//  * @param token - 消息投递令牌，包含消息ID等元数据
//  */
// void callback::delivery_complete(mqtt::delivery_token_ptr token)
// {
//     // 记录消息投递完成日志，包含令牌对应的消息ID
//     logging::get_logger()->info("Delivery complete for token: [{}]", token->get_message_id());
// }

// /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

// /**
//  * @beilf:处理上传任务的命令（当前为空实现）
//  * @param json_str - 包含任务数据的JSON字符串
//  */
// void handle_upload_mission(const std::string &json_str)
// {
//     // 当前函数为空实现，需补充任务解析和上传逻辑
//     // 应从json_str中提取航点数据并发送给无人机
// }

// /**
//  * @beilf:根据接收到的任务ID执行相应操作，并返回执行结果
//  * @param missionId - 任务ID（字符串类型，标识不同任务类型）
//  * @param context - 无人机上下文对象（包含各控制插件）
//  * @param waypoint_route - 航点路径（用于航线任务）
//  * @return 0 - 操作成功
//  * @return 非0 - 操作失败（具体含义由fly_mission函数定义）
//  */
// int process_start_mission(const std::string &missionId, Mavsdk_members &mavsdk, const std::string &waypoint_route)
// {
//     // 任务ID为"1"：启动/停止目标检测功能
//     if (missionId == "1")
//     {
//         // 使用原子操作切换检测状态（0→1或1→0）
//         start_detect.store(start_detect.load() ? 0 : 1);
//         logging::get_logger()->info("start_detect: {}", start_detect.load());
//         return 0;
//     }
//     // 任务ID为"21"：预留功能（当前未使用）
//     else if (missionId == "21")
//     {
//         find_landmark.store(find_landmark.load() ? 0 : 1);
//         logging::get_logger()->info("find_landmark: {}", find_landmark.load());
//         return 0;
//     }
//     // 任务ID为"2"：预留功能（当前未使用）
//     else if (missionId == "2")
//     {
//         arming_and_takeoff(mavsdk, 5.0, 30);

//         // start_posctl.store(start_posctl.load() ? 0 : 1);
//         // logging::get_logger()->info("start_posctl: {}", start_posctl.load());
//         return 0;
//     }
//     // 任务ID为"3"：预留功能（当前未使用）
//     else if (missionId == "3")
//     {
//         offboard_flight_position(mavsdk, 1.0, 1.0, -6.0, 90);

//         // is_take_off.store(is_take_off.load() ? 0 : 1);
//         // logging::get_logger()->info("is_take_off: {}", is_take_off.load());
//         return 0;
//     }
//     // 任务ID为"4"：启动/停止目标检测线程
//     else if (missionId == "4")
//     {
//         detection_running.store(detection_running.load() ? false : true);
//         logging::get_logger()->info("detection_running: {}", detection_running.load());
//         return 0;
//     }

//     // 其他任务ID：调用fly_mission执行航线任务
//     int reply_code = fly_mission(missionId, mavsdk, waypoint_route);
//     return reply_code;
// }

// /**
//  * @beilf:处理启动任务的JSON命令解析函数  从JSON字符串中提取任务参数，并调用任务处理函数
//  * @param json_str - JSON格式的命令字符串
//  * @param context - 无人机上下文对象（包含各控制插件）
//  */
// void handle_start_mission(const std::string &json_str, Mavsdk_members &mavsdk)
// {
//     try
//     {
//         std::string missionId;      // 任务ID
//         std::string waypoint_route; // 航点路径（航线任务使用）

//         // 解析JSON字符串
//         auto root = nlohmann::json::parse(json_str);

//         // 检查JSON结构是否包含"data"对象
//         if (root.contains("data") && root["data"].is_object())
//         {
//             auto data = root["data"];

//             // 提取任务ID
//             if (data.contains("missionId") && data["missionId"].is_string())
//             {
//                 missionId = data["missionId"];
//             }
//             else
//             {
//                 // 缺少missionId字段时直接返回
//                 return; // 从函数中提前返回，终止函数的执行流程
//             }

//             // 提取航点路径（对航线任务必需）
//             if (data.contains("waypointroute") && data["waypointroute"].is_string())
//             {
//                 waypoint_route = data["waypointroute"];
//             }
//             else
//             {
//                 // 缺少waypointroute字段时直接返回
//                 return;
//             }

//             // 调用任务处理函数（传递任务ID和航点路径）
//             process_start_mission(missionId, mavsdk, waypoint_route);
//         }
//     }
//     catch (const nlohmann::json::parse_error &e)
//     {
//         // 捕获JSON解析异常并记录错误日志
//         logging::get_logger()->error("JSON parse error: {}", e.what());
//     }
// }

// /**
//  * @beilf:处理起飞降落命令的核心函数
//  * @param enable - 操作类型（0:降落 1:起飞）
//  * @param context - 无人机上下文（包含控制和遥测模块）
//  * @return 0 - 操作成功
//  * @return 非0 - 操作失败（具体错误码由takeoff_and_land定义）
//  */
// int process_takeoff_land(int enable, Mavsdk_members &mavsdk)
// {
//     // 调用底层起飞降落实现函数
//     // 传递操作类型和所需的控制/遥测模块
//     int reply_code = takeoff_and_land(enable, mavsdk);
//     return reply_code;
// }

// /**
//  * @beilf:处理起飞降落的JSON命令解析函数
//  * @param json_str - 包含起飞降落命令的JSON字符串
//  * @param context - 无人机上下文对象
//  */
// void handle_takeoff_land(const std::string &json_str, Mavsdk_members &mavsdk)
// {
//     try
//     {
//         // 解析JSON字符串
//         auto root = nlohmann::json::parse(json_str);

//         // 验证JSON结构
//         if (root.contains("data") && root["data"].is_object())
//         {
//             auto data = root["data"];

//             // 提取并验证enable参数（必须为整数类型）
//             if (data.contains("enable") && data["enable"].is_number_integer())
//             {
//                 int enable = data["enable"];

//                 // 调用起飞降落处理函数
//                 process_takeoff_land(enable, mavsdk);
//             }
//         }
//     }
//     catch (const nlohmann::json::parse_error &e)
//     {
//         // 记录JSON解析错误
//         logging::get_logger()->error("JSON parse error: {}", e.what());
//     }
// }

// /**
//  * @beilf:处理自动降落功能的启用/禁用 配置高度测量并订阅高度数据，为自动降落做准备
//  * @param enable - 功能启用标志（非0值启用，0禁用）
//  * @param context - 无人机上下文对象（包含遥测模块）
//  * @return 0 - 操作成功
//  * @return 1 - 设置高度测量频率失败
//  */
// int process_enable_auto_landing(int enable, Mavsdk_members &mavsdk)
// {
//     // 记录进入高度测量的日志
//     logging::get_logger()->info("进入高度测量");

//     // 设置位置数据更新频率为1Hz（每秒1次）
//     const auto set_rate_result = mavsdk.telemetry.set_rate_position(1.0);
//     if (set_rate_result != Telemetry::Result::Success)
//     {
//         // 频率设置失败时记录错误
//         logging::get_logger()->info("设置高度测量频率失败");
//         std::cerr << "Setting rate failed: " << set_rate_result << '\n';
//         return 1;
//     }

//     // 订阅无人机位置数据（包含高度信息）
//     auto position_handle = mavsdk.telemetry.subscribe_position(
//         [](Telemetry::Position position)
//         {
//             // 回调函数：打印当前相对高度
//             logging::get_logger()->info("当前高度：{}", position.relative_altitude_m);
//         });

//     return 0;
// }

// /**
//  * @beilf:处理自动降落功能的JSON命令解析
//  * @param json_str - 包含命令的JSON字符串
//  * @param context - 无人机上下文对象
//  */
// void handle_enable_auto_landing(const std::string &json_str, Mavsdk_members &mavsdk)
// {
//     try
//     {
//         // 解析JSON字符串
//         auto root = nlohmann::json::parse(json_str);

//         // 验证JSON结构是否包含"data"对象
//         if (root.contains("data") && root["data"].is_object())
//         {
//             auto data = root["data"];

//             // 提取并验证enable参数（必须为整数类型）
//             if (data.contains("enable") && data["enable"].is_number_integer())
//             {
//                 int enable = data["enable"];

//                 // 调用自动降落处理函数
//                 process_enable_auto_landing(enable, mavsdk);
//             }
//         }
//     }
//     catch (const nlohmann::json::parse_error &e)
//     {
//         // 捕获JSON解析异常并记录错误
//         logging::get_logger()->error("JSON parse error: {}", e.what());
//     }
// }

// /**
//  * @beilf:处理启动目标检测的命令
//  * @param enable - 启用标志（1:启动检测 0:停止检测）
//  * @param context - 无人机上下文（包含相机、遥测等模块）
//  * @return 0 - 操作成功（当前实现为空）
//  */
// int process_start_detect(int enable, Mavsdk_members &mavsdk)
// {
//     // 当前函数为空实现，需补充实际检测逻辑
//     // 应根据enable值控制检测线程的启动/停止

//     return 0;
// }

// /**
//  * @beilf:处理启动目标检测的JSON命令解析
//  * @param json_str - 包含检测命令的JSON字符串
//  * @param context - 无人机上下文
//  */
// void handle_start_detect(const std::string &json_str, Mavsdk_members &mavsdk)
// {
//     try
//     {
//         // 解析JSON字符串
//         auto root = nlohmann::json::parse(json_str);

//         // 验证JSON结构
//         if (root.contains("data") && root["data"].is_object())
//         {
//             auto data = root["data"];

//             // 提取并验证enable参数
//             if (data.contains("enable") && data["enable"].is_number_integer())
//             {
//                 int enable = data["enable"];

//                 // 调用检测处理函数
//                 process_start_detect(enable, mavsdk);
//             }
//         }
//     }
//     catch (const nlohmann::json::parse_error &e)
//     {
//         // 记录JSON解析错误
//         logging::get_logger()->error("JSON parse error: {}", e.what());
//     }
// }

// /**
//  * @beilf:解析JSON命令并分发到对应处理函数
//  * @param json - JSON格式的命令字符串
//  * @param context - 无人机上下文对象（包含各控制模块）
//  */
// void process_command(const std::string &json, Mavsdk_members &mavsdk)
// {
//     try
//     {
//         // 解析JSON字符串
//         auto j = nlohmann::json::parse(json);

//         // 记录scopeId（命令作用域标识）
//         if (j.contains("scopeId") && j["scopeId"].is_string())
//         {
//             logging::get_logger()->info("scopeId: {}", j["scopeId"].get<std::string>());
//         }

//         // 提取并处理method字段（命令类型）
//         if (j.contains("method") && j["method"].is_string())
//         {
//             std::string method = j["method"];

//             // 根据method字段值分发到不同的处理函数
//             if (method == "upload_mission")
//             {
//                 handle_upload_mission(json); // 上传飞行任务
//             }
//             else if (method == "start_mission")
//             {
//                 handle_start_mission(json, mavsdk); // 启动飞行任务
//             }
//             else if (method == "takeoff_land")
//             {
//                 handle_takeoff_land(json, mavsdk); // 起飞/降落控制
//             }
//             else if (method == "enable_auto_landing")
//             {
//                 handle_enable_auto_landing(json, mavsdk); // 启用自动降落
//             }
//             else if (method == "start_detect")
//             {
//                 handle_start_detect(json, mavsdk); // 启动目标检测
//             }
//             else
//             {
//                 // 未知命令类型记录警告日志
//                 logging::get_logger()->warn("Unknown method: {}", method);
//             }
//         }
//     }
//     catch (const nlohmann::json::parse_error &e)
//     {
//         // 捕获JSON解析异常并记录错误
//         logging::get_logger()->error("JSON parse error: {}", e.what());
//     }
// }

// // /**
// //  * @beilf:建立与无人机的通信连接
// //  * @param drone_sdk - Mavsdk SDK实例
// //  * @param connection_url - 连接URL（如"udp://:14540"）
// //  * @return 0 - 连接成功
// //  * @return-1 - 连接失败
// //  */
// // int px4_communication_connection(Mavsdk &drone_sdk, const std::string &connection_url)
// // {
// //     // 尝试添加连接（支持多种连接方式：UDP/TCP/Serial等）
// //     ConnectionResult connection_result = drone_sdk.add_any_connection(connection_url);

// //     // 检查连接结果
// //     if (connection_result != ConnectionResult::Success)
// //     {
// //         // 连接失败时记录错误日志并发布MQTT消息
// //         mqtt_client::publish(PX4_REPLY, "连接 PX4 失败");
// //         return -1;
// //     }

// //     return 0; // 连接成功返回0
// // }

// // /*::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

// // int main(int argc, char *argv[])
// // {
// //     // 解析MQTT服务器地址，默认使用本地地址
// //     auto serverURI = (argc > 1) ? std::string{argv[1]} : DFLT_SERVER_URI;

// //     // 创建MQTT客户端，连接到指定服务器并设置客户端ID
// //     mqtt::async_client cli(serverURI, CLIENT_ID);

// //     // 配置MQTT连接选项
// //     mqtt::connect_options connOpts;
// //     connOpts.set_clean_session(false);                           // 保留会话状态
// //     connOpts.set_keep_alive_interval(std::chrono::seconds(300)); // 保持连接间隔
// //     connOpts.set_user_name(MQTTUSER);                            // 设置用户名
// //     connOpts.set_password(MQTTPASSWORD);                         // 设置密码

// //     // 设置MQTT回调处理类，用于处理连接状态和消息接收
// //     callback cb(cli, connOpts);
// //     cb.set_reply_message("机载电脑mqtt已经连接上了服务器");
// //     cli.set_callback(cb);

// //     try
// //     {
// //         // 尝试连接到MQTT服务器
// //         logging::get_logger()->info("Connecting to the MQTT server '{}'...", serverURI);
// //         cli.connect(connOpts, nullptr, cb);
// //     }
// //     catch (const mqtt::exception &exc)
// //     {
// //         // 连接失败处理
// //         logging::get_logger()->error("ERROR: Unable to connect to MQTT server: '{}' {}", serverURI, exc.what());
// //         return 1;
// //     }

// //     // 初始化MQTT客户端（全局单例模式）
// //     mqtt_client::initialize(&cli);
// //     logging::get_logger()->info("已经生成全局Mqtt对象");

// //     // 根据编译选项选择连接方式（真实硬件或仿真环境）
// // #ifdef REAL_HARDWARE
// //     std::string connection_url = "serial:///dev/ttyACM0:921600"; // 真实硬件通过串口连接

// // #elif defined(SIMULATION)
// //     std::string connection_url = "udpin://0.0.0.0:14540";     // 仿真环境通过UDP连接
// //     GazeboCamera::Instance()->init(argc, argv, subscribePtr); // 初始化Gazebo仿真相机

// // #else
// //     mqtt_client::publish(PX4_REPLY, "模式(真机/仿真)定义错误");

// // #endif

// //     // 创建MAVSDK实例并配置为地面站类型
// //     Mavsdk drone_sdk{Mavsdk::Configuration{ComponentType::GroundStation}};

// //     // 建立与无人机的通信连接
// //     int connect_status = px4_communication_connection(drone_sdk, connection_url);
// //     if (connect_status != 0)
// //     {
// //         // 连接失败处理
// //         mqtt_client::publish(PX4_REPLY, "连接状态错误");
// //     }

// //     // 等待自动飞行器系统出现（超时时间3秒）
// //     auto system = drone_sdk.first_autopilot(5.0);
// //     if (!system) // 超时处理
// //     {
// //         mqtt_client::publish(PX4_REPLY, "无人机连接等待超时");
// //         std::this_thread::sleep_for(std::chrono::seconds(5));
// //         return 1;
// //     }

// //     // 创建各种无人机功能模块的可选实例
// //     // 声明了一个全局变量 g_mavlinkpassthrough，它的类型是 std::optional<MavlinkPassthrough>
// //     // std::optional 用于表示一个可能存在的值，类似于一个轻量级的容器，要么包含一个值，要么为空（std::nullopt）
// //     std::optional<MavlinkPassthrough> g_mavlinkpassthrough; // MAVLink透传
// //     std::optional<MissionRaw> g_mission_raw;                // 航线任务接口
// //     std::optional<Telemetry> g_telemetry;                   // 遥测数据
// //     std::optional<Offboard> g_offboard;                     // 外部控制模式
// //     std::optional<Mission> g_mission;                       // 航线任务规划
// //     std::optional<Action> g_action;                         // 飞行控制动作（起飞、降落等）
// //     std::optional<Camera> g_camera;                         // 相机控制

// //     // 如果连接到飞控系统，初始化所有功能模块
// //     if (system.has_value())
// //     {
// //         // 系统连接成功
// //         mqtt_client::publish(PX4_REPLY, "无人机连接成功");

// //         // 获取已发现的无人机系统的引用（使用value()方法获取optional中的值）
// //         auto &system_ref = system.value();

// //         // 使用emplace方法直接在 optional 内部构造对象，避免不必要的拷贝或移动操作
// //         // 如果 optional 为空：直接在其内部空间构造对象
// //         // 如果 optional 已有值：先销毁当前值，再构造新值
// //         g_mavlinkpassthrough.emplace(system_ref); // 初始化MAVLink透传插件（用于发送自定义MAVLink消息）
// //         g_mission_raw.emplace(system_ref);        // 初始化原始任务插件（用于处理MAVLink原生任务格式）
// //         g_telemetry.emplace(system_ref);          // 初始化遥测数据插件（用于获取无人机状态信息）
// //         g_mission.emplace(system_ref);            // 初始化任务规划插件（用于加载和执行预定义任务）
// //         g_offboard.emplace(system_ref);           // 初始化外部控制插件（用于发送实时控制命令）
// //         g_action.emplace(system_ref);             // 初始化飞行控制插件（用于起飞、降落、悬停等操作）
// //         g_camera.emplace(system_ref);             // 初始化相机控制插件（用于控制无人机相机）
// //     }
// //     else
// //     {
// //         mqtt_client::publish(PX4_REPLY, "无人机连接失败");
// //     }

// //     // 创建Mavsdk对象，整合所有MAVSDK功能模块
// //     // 该对象用于在整个程序中传递和共享无人机相关功能
// //     Mavsdk_members mavsdk{
// //         g_mavlinkpassthrough.value(), // MAVLink透传模块：用于发送和接收原始MAVLink消息
// //         g_mission_raw.value(),        // 原始任务模块：提供对MAVLink原生任务格式的支持
// //         g_telemetry.value(),          // 遥测数据模块：提供无人机状态信息（位置、姿态、速度等）
// //         g_offboard.value(),           // 外部控制模块：用于发送实时控制命令（如位置、速度指令）
// //         g_mission.value(),            // 任务规划模块：用于加载、执行和管理预定义的飞行任务
// //         g_action.value(),             // 飞行控制动作模块：用于起飞、降落、悬停等基本飞行操作
// //         g_camera.value()              // 相机控制模块：用于控制无人机搭载的相机（拍照、录像等）
// //     };

// //     // 将MAVSDK功能模块设置到MQTT回调中，便于消息处理时访问无人机功能
// //     cb.set_context(mavsdk);

// //     // 主循环：等待用户输入'q'退出程序
// //     while (std::tolower(std::cin.get()) != 'q')
// //         ;

// //     // 程序退出前的清理工作
// //     try
// //     {
// //         cli.disconnect()->wait(); // 断开MQTT连接
// //     }
// //     catch (const mqtt::exception &exc)
// //     {
// //         return 1;
// //     }

// //     return 0;
// // }