#include "mqtt_client.hpp"
#include "state_machine.hpp"

#include <filesystem>    // C++17文件系统库，用于目录创建和文件操作
#include <fstream>       // 添加文件流头文件
#include <iomanip>       // 格式化输出
#include <iostream>      // 输入输出
#include <openssl/md5.h> // MD5校验和计算
#include <sstream>       // 字符串流

// 统一使用命名空间别名
using namespace std;
using namespace mqtt_ns;        // 引入MQTT库别名
namespace fs = std::filesystem; // 命名空间简化

/**
 * @brief 构造函数实现
 * @param saveDir 文件保存目录路径
 */
Mqtt::Mqtt(const std::string &saveDir) : saveDir(saveDir), client(BROKER, CLIENT_ID)
{
    // 创建目录（如果不存在）
    createDirectory(saveDir);
}

/**
 * @brief 初始化MQTT连接
 * @return 连接成功返回true，失败返回false
 */
bool Mqtt::init()
{
    // 配置MQTT连接选项
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(KEEP_ALIVE); // 设置心跳间隔
    connOpts.set_user_name(USERNAME);             // 设置用户名
    connOpts.set_password(PASSWORD);              // 设置密码
    connOpts.set_clean_session(true);             // 设置为清除会话模式

    // 设置回调处理类（当前类）
    client.set_callback(*this);

    try
    {
        mqtt::token_ptr conntok = client.connect(connOpts); // 同步连接到MQTT代理服务器
        conntok->wait();                                    // 等待连接完成
        sendMessage(REPLAY_TOPIC, "MQTT连接成功");          // 发送连接成功消息
        return true;
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "MQTT连接失败: " << exc.what() << std::endl;
        return false;
    }
}

/**
 * @brief MQTT连接丢失
 * @param cause 连接丢失原因描述
 */
void Mqtt::connection_lost(const std::string &cause)
{
    std::cout << "MQTT连接丢失: " << cause << std::endl;
    std::cout << "尝试重新连接..." << std::endl;

    while (!client.is_connected())
    {
        try
        {
            client.reconnect()->wait();
            std::cout << "重新连接成功" << std::endl;
            sendMessage(REPLAY_TOPIC, "mqtt重新连接成功");

            // 重新订阅所有主题
            std::lock_guard<std::mutex> lock(topicMutex);
            for (const auto &topic : subscribedTopics)
            {
                client.subscribe(topic, 0)->wait();
            }
        }
        catch (const mqtt::exception &exc)
        {
            std::cerr << "重新连接失败: " << exc.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(5));
        }
    }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::: MQTT 接收函数 :::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/**
 * @brief MQTT消息到达
 * @param msg 接收到的消息指针
 */
void Mqtt::message_arrived(const_message_ptr msg)
{
    lock_guard<mutex> lock(fileMutex);                                                   // 使用互斥锁保护文件操作，确保线程安全
    string topic = msg->get_topic();                                                     // 提取消息的主题（Topic）
    vector<unsigned char> payload(msg->get_payload().begin(), msg->get_payload().end()); // 提取消息的二进制载荷
    cout << "\n收到消息: " << topic << endl;                                             // 打印接收到的主题信息

    /*:::::::::::::::::::::::::::::::::::: 不同主题任务处理 :::::::::::::::::::::::::::::::::::::;*/
    if (topic == "test")
    {
        handle_Test_Message(payload);
        return;
    }
    /*:::::::::::::::::::::::::::::::::::: 不同主题任务处理 :::::::::::::::::::::::::::::::::::::;*/

    // 解析主题结构
    size_t pos = topic.find_last_of('/'); // 查找主题中最后一个'/'的位置
    if (pos == std::string::npos)         // 如果未找到'/'，说明主题格式无效，直接返回
        return;

    // 拆分主题为两部分：
    // - subTopic：最后一个'/'之后的部分（例如"meta"或"123"）
    // - baseTopic：最后一个'/'之前的部分（例如"transferfiles"或"transferfiles/data"）
    std::string subTopic = topic.substr(pos + 1); // 提取最后一部分（子主题）
    std::string baseTopic = topic.substr(0, pos); // 提取前缀部分（基础主题）

    // 处理元数据消息（格式：transferfiles/meta）
    if (subTopic == "meta")
    {
        std::string payloadStr(payload.begin(), payload.end()); // 将二进制载荷转换为字符串（元数据通常为JSON格式）
        handleMetaMessage(payloadStr);                          // 调用航线文件处理函数
        return;                                                 // 处理完成后提前返回
    }

    // 处理数据块消息（格式：transferfiles/data/123）
    if (baseTopic.find("/data") != std::string::npos)
    {
        try
        {
            int chunkId = std::stoi(subTopic); // 将子主题（subTopic）解析为整数块ID（如"123"转为123）

            // 调用数据块处理函数（如保存块数据、更新接收进度）
            handleDataMessage(chunkId, payload);
        }
        catch (const std::invalid_argument &)
        {
            std::cerr << "无效的块ID: " << subTopic << std::endl;
        }
    }
}

// 处理测试消息
void Mqtt::handle_Test_Message(const std::vector<unsigned char> &payload)
{
    try
    {
        std::string payloadStr(payload.begin(), payload.end());
        json msg = json::parse(payloadStr);

        // 提取JSON字段
        std::string content = msg.value("msg", "");

        // 处理消息
        std::cout << "收到test主题消息: " << content << std::endl;

        /*::::::::::::::::::::::::::::: 添加自定义业务逻辑 :::::::::::::::::::::::::::::::::::*/
        if (content == "hello")
        {
            start_machine_flag.flag = true; // 设置全局状态机标志为true
            sendMessage(REPLAY_TOPIC, "world!");
        }
        /*::::::::::::::::::::::::::::: 添加自定义业务逻辑 :::::::::::::::::::::::::::::::::::*/
    }
    catch (const json::parse_error &e)
    {
        std::cerr << "解析test消息失败: " << e.what() << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "处理test消息失败: " << e.what() << std::endl;
    }
}

/**
 * @brief 订阅单个主题（带线程安全）
 */
void Mqtt::subscribeTopic(const std::string &topic)
{
    // 使用互斥锁保护共享资源（订阅主题集合），确保线程安全
    std::lock_guard<std::mutex> lock(topicMutex);

    // 检查主题是否已被订阅（避免重复订阅）
    if (subscribedTopics.count(topic) == 0)
    {
        try
        {
            // 订阅主题
            client.subscribe(topic, 0)->wait(); // wait()方法会阻塞直到订阅操作完成

            // 将成功订阅的主题添加到集合中
            subscribedTopics.insert(topic);

            // 打印成功订阅的信息
            std::cout << "已订阅主题: " << topic << std::endl;
        }
        catch (const mqtt::exception &exc)
        {
            // 捕获并处理订阅异常
            std::cerr << "订阅主题失败: " << topic << " - " << exc.what() << std::endl;
        }
    }
}

/**
 * @brief 订阅多个主题
 */
void Mqtt::subscribeTopics(const std::vector<std::string> &topics)
{
    for (const auto &topic : topics)
    {
        subscribeTopic(topic);
    }
}
/*:::::::::::::::::::::::::::::::::::::::::::::::: MQTT 接收函数 :::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*:::::::::::::::::::::::::::::::::::::::::::::: MQTT 文件接受处理 ::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/**
 * @brief 创建目录（如果不存在）
 * @param path 目录路径
 */
void Mqtt::createDirectory(const std::string &path)
{
    try
    {
        if (!fs::exists(path))
        {
            fs::create_directories(path); // 递归创建目录
            std::cout << "创建目录: " << path << std::endl;
        }
    }
    catch (const fs::filesystem_error &e)
    {
        std::cerr << "创建目录失败: " << e.what() << std::endl;
    }
}

/**
 * @brief 监听航线文件MQTT消息
 */
void Mqtt::Monitor_flight_file()
{
    try
    {
        // 订阅指定主题（QoS级别0：最多一次传递）
        client.subscribe(TOPIC, 0)->wait(); // wait()方法会阻塞直到订阅操作完成

        // 打印成功订阅的信息
        std::cout << "已订阅主题: " << TOPIC << std::endl;
        std::cout << "等待航线文件传输" << std::endl;

        // 保持主线程运行（使用休眠避免CPU占用过高）
        // 此循环确保程序不会退出，以便持续接收MQTT消息
        while (true)
        {
            // 休眠1秒，减少CPU占用
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    catch (const mqtt::exception &exc)
    {
        // 捕获并处理MQTT操作异常
        std::cerr << "MQTT操作失败: " << exc.what() << std::endl;
    }
}

/**
 * @brief 计算文件的MD5校验和
 * @param filePath 文件路径
 * @return MD5字符串（小写十六进制格式）
 */
std::string Mqtt::calculateMD5(const std::string &filePath)
{
    std::ifstream file(filePath, std::ios::binary);
    if (!file)
        return ""; // 文件打开失败

    unsigned char digest[MD5_DIGEST_LENGTH];
    MD5_CTX ctx;
    MD5_Init(&ctx);

    const int bufferSize = 8192;
    char buffer[bufferSize];

    // 分块读取文件内容计算MD5
    while (file)
    {
        file.read(buffer, bufferSize);
        std::streamsize bytes = file.gcount();
        if (bytes > 0)
        {
            MD5_Update(&ctx, buffer, bytes);
        }
    }

    MD5_Final(digest, &ctx);

    // 转换为十六进制字符串表示
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (int i = 0; i < MD5_DIGEST_LENGTH; ++i)
    {
        oss << std::setw(2) << static_cast<int>(digest[i]);
    }

    return oss.str();
}

/**
 * @brief 处理元数据消息
 * @param payload JSON格式的元数据字符串
 */
void Mqtt::handleMetaMessage(const std::string &payload)
{
    try
    {
        json meta = json::parse(payload); // 解析JSON元数据

        // 创建新的文件信息对象
        currentFile = std::make_unique<FileInfo>();
        currentFile->name = meta["name"].get<std::string>(); // 文件名
        currentFile->size = meta["size"].get<size_t>();      // 文件大小
        currentFile->chunks = meta["chunks"].get<size_t>();  // 总分块数
        currentFile->received = 0;                           // 已接收块数
        currentFile->data.resize(currentFile->chunks);       // 预分配块存储空间
        currentFile->isLast = meta.value("is_last", false);  // 是否为最后一个文件
        currentFile->md5 = meta.value("md5", "");            // 文件MD5校验和

        std::cout << "开始接收文件: " << currentFile->name << " (" << currentFile->chunks << "块)" << std::endl;
    }
    catch (const json::parse_error &e)
    {
        std::cerr << "解析元数据失败: " << e.what() << std::endl;
        currentFile.reset(); // 重置当前文件信息
    }
    catch (const std::exception &e)
    {
        std::cerr << "处理元数据失败: " << e.what() << std::endl;
        currentFile.reset(); // 重置当前文件信息
    }
}

/**
 * @brief 处理数据块消息
 * @param chunkId 块ID
 * @param payload 块数据内容
 */
void Mqtt::handleDataMessage(int chunkId, const std::vector<unsigned char> &payload)
{
    if (!currentFile)
        return; // 没有活动文件，忽略

    // 检查块ID有效性
    if (chunkId < 0 || static_cast<size_t>(chunkId) >= currentFile->chunks)
    {
        std::cerr << "无效的块ID: " << chunkId << std::endl;
        return;
    }

    // 检查块是否已接收（可能是重复发送的块）
    if (!currentFile->data[chunkId].empty())
    {
        std::cout << "警告: 块 " << chunkId << " 已存在，正在覆盖" << std::endl;
    }

    // 保存数据块
    currentFile->data[chunkId] = payload;
    currentFile->received++;

    // 检查是否所有块都已接收
    if (currentFile->received == currentFile->chunks)
    {
        std::cout << std::endl; // 换行
        saveCurrentFile();      // 保存完整文件
    }
}

/**
 * @brief 保存当前文件
 */
void Mqtt::saveCurrentFile()
{
    if (!currentFile)
        return; // 没有活动文件，忽略

    std::string filePath = saveDir + "/" + currentFile->name;
    std::ofstream file(filePath, std::ios::binary);

    if (!file)
    {
        std::cerr << "无法创建文件: " << filePath << std::endl;
        sendAck(currentFile->name, "fail"); // 发送失败确认
        return;
    }

    // 按顺序写入所有数据块
    for (const auto &chunk : currentFile->data)
    {
        file.write(reinterpret_cast<const char *>(chunk.data()), chunk.size());
    }

    file.close();
    std::cout << "文件已保存: " << filePath << std::endl; // 提示文件保存的路径

    // 验证MD5校验和
    std::string status = "ok";
    if (!currentFile->md5.empty())
    {
        std::string localMd5 = calculateMD5(filePath);
        if (localMd5 == currentFile->md5)
        {
            std::cout << "MD5验证成功" << std::endl;
        }
        else
        {
            std::cout << "MD5验证失败" << std::endl;
            std::cout << "本地MD5: " << localMd5 << std::endl;
            std::cout << "期望MD5: " << currentFile->md5 << std::endl;
            status = "fail";

            fs::remove(filePath); // 删除校验失败的文件
        }
    }

    sendAck(currentFile->name, status); // 发送接收确认消息

    // 检查是否是最后一个文件
    if (currentFile->isLast)
    {
        std::cout << "所有文件接收完成!" << std::endl;
    }

    // 重置当前文件信息
    currentFile.reset();
}
/*:::::::::::::::::::::::::::::::::::::::::::::: MQTT 文件接受处理 ::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/*:::::::::::::::::::::::::::::::::::::::::::::::: MQTT 发送函数 :::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
/**
 * @brief 发送单条MQTT消息（线程安全）
 * @param topic 目标主题
 * @param payload 消息内容（字符串）
 * @param qos 服务质量等级（0/1/2）
 * @return 发送成功返回true，失败返回false
 */
bool Mqtt::sendMessage(const string &topic, const string &payload)
{
    lock_guard<mutex> lock(sendMutex);
    if (!client.is_connected())
    {
        cerr << "未连接到MQTT代理,无法发送消息到: " << topic << endl;
        return false;
    }

    try
    {
        mqtt::binary_ref binPayload(payload.data(), payload.size());

        // 创建消息对象（参数：主题、载荷、QoS、保留标志、属性）
        auto msg = make_shared<mqtt::message>(topic, binPayload, 0, false, mqtt::properties());

        // 异步发送并等待完成
        client.publish(msg)->wait();
        // cout << "消息已发送到 [" << topic << "]: " << payload << endl;
        return true;
    }
    catch (const std::exception &e)
    {
        cerr << "发送消息失败: " << e.what() << endl;
        return false;
    }
}

/**
 * @brief 批量发送MQTT消息（线程安全）
 * @param messages 消息列表（主题-内容对）
 * @param qos 服务质量等级（0/1/2）
 * @return 成功发送的消息数量
 */
size_t Mqtt::sendMessages(const vector<pair<string, string>> &messages)
{
    size_t successCount = 0;

    for (const auto &msg : messages)
    {
        if (sendMessage(msg.first, msg.second))
        {
            successCount++;
        }
    }

    return successCount;
}

/**
 * @brief 发送接收确认消息
 * @param fileName 文件名
 * @param status 状态（"ok"或"fail"）
 */
void Mqtt::sendAck(const std::string &fileName, const std::string &status)
{
    try
    {
        // 检查MQTT客户端连接状态
        if (!client.is_connected())
        {
            std::cerr << "MQTT客户端未连接,无法发送确认消息" << std::endl;
            return;
        }

        // 构造确认消息JSON
        json ack = {
            {"name", fileName},
            {"status", status}};

        // 构造MQTT消息（QoS 0，非保留消息）
        mqtt::message msg(ACK_TOPIC, ack.dump(), 0, false);

        // 使用智能指针发送消息
        auto msg_ptr = std::make_shared<const mqtt::message>(msg);
        client.publish(msg_ptr);
    }
    catch (const std::exception &e)
    {
        std::cerr << "发送确认消息失败: " << e.what() << std::endl;
    }
}

/*:::::::::::::::::::::::::::::::::::::::::::::::: MQTT 发送函数 :::::::::::::::::::::::::::::::::::::::::::::::::::::::*/
