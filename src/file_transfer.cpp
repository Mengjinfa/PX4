#include "file_transfer.hpp"
#include "mqtt_client.hpp"

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <openssl/md5.h>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

std::mutex fileMutex;                      // 文件操作互斥锁
std::shared_ptr<FileMetadata> currentFile; // 当前传输的文件
std::string fileSaveDir;                   // 文件保存目录

namespace fs = std::filesystem;

/*::::::::::::::::::::::::::::::::::::::::::::::::::::: 传输文件相关代码 :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::*/

/**
 * @brief 设置文件保存目录
 */
void setFileSaveDirectory(const std::string &dir)
{
    std::lock_guard<std::mutex> lock(fileMutex);
    fileSaveDir = dir;
    fs::create_directories(fileSaveDir);
    std::cout << "文件保存目录已设置为: " << fileSaveDir << std::endl;
}

/**
 * @brief 简化的JSON解析函数
 */
std::unordered_map<std::string, std::string> parseJson(const std::string &jsonStr)
{
    std::unordered_map<std::string, std::string> result;

    size_t pos = 1;                  // 跳过开头的 '{'
    while (pos < jsonStr.size() - 1) // 减去结尾的 '}'
    {

        size_t keyStart = jsonStr.find('"', pos) + 1; // 查找键的开始位置
        size_t keyEnd = jsonStr.find('"', keyStart);
        if (keyStart == std::string::npos || keyEnd == std::string::npos)
        {
            break;
        }

        std::string key = jsonStr.substr(keyStart, keyEnd - keyStart);

        size_t valStart = jsonStr.find(':', keyEnd) + 1;                                             // 查找值的开始位置
        while (valStart < jsonStr.size() && (jsonStr[valStart] == ' ' || jsonStr[valStart] == '\t')) // 跳过可能的空格
        {
            valStart++;
        }

        // 确定值的类型
        std::string value;
        if (jsonStr[valStart] == '"') // 字符串值
        {
            valStart++;                                  // 跳过开头的 '"'
            size_t valEnd = jsonStr.find('"', valStart); // 查找值的结束位置
            if (valEnd == std::string::npos)
            {
                break;
            }

            value = jsonStr.substr(valStart, valEnd - valStart);
            pos = valEnd + 1;
        }
        else if (jsonStr[valStart] == '{' || jsonStr[valStart] == '[') // 对象或数组值
        {

            char endChar = jsonStr[valStart] == '{' ? '}' : ']';
            size_t valEnd = jsonStr.find(endChar, valStart + 1);
            if (valEnd == std::string::npos)
            {
                break;
            }

            value = jsonStr.substr(valStart, valEnd - valStart + 1);
            pos = valEnd + 1;
        }
        else // 数字或布尔值
        {
            size_t valEnd = jsonStr.find_first_of(",}]", valStart);
            if (valEnd == std::string::npos)
            {
                valEnd = jsonStr.size() - 1;
            }

            value = jsonStr.substr(valStart, valEnd - valStart);
            pos = valEnd;
        }

        // 跳过逗号
        if (pos < jsonStr.size() && jsonStr[pos] == ',')
        {
            pos++;
        }

        result[key] = value;
    }

    return result;
}

/**
 * @brief 处理文件元数据
 */
void processFileMetadata(const std::string &metadataStr)
{
    // 简化的JSON解析
    std::unordered_map<std::string, std::string> meta = parseJson(metadataStr);
    if (meta.find("name") == meta.end() || meta.find("size") == meta.end() || meta.find("chunks") == meta.end())
    {
        std::cerr << "无效的文件元数据" << std::endl;
        return;
    }

    // 创建新的文件元数据结构
    currentFile = std::make_shared<FileMetadata>();
    currentFile->name = meta["name"];
    currentFile->size = std::stoul(meta["size"]);
    currentFile->chunks = std::stoi(meta["chunks"]);
    currentFile->received_chunks = 0;
    currentFile->data.resize(currentFile->chunks);
    currentFile->is_last = (meta.find("is_last") != meta.end() && meta["is_last"] == "true");
    currentFile->md5 = meta.find("md5") != meta.end() ? meta["md5"] : "";

    std::cout << "\n开始接收文件: " << currentFile->name
              << " (大小: " << currentFile->size
              << " 字节, 块数: " << currentFile->chunks << ")" << std::endl;
}

/**
 * @brief 发送文件接收确认
 */
void sendFileAck(bool success)
{
    try
    {
        std::string status = success ? "ok" : "fail";                                                      // 状态字符串
        std::string ackPayload = "{\"name\":\"" + currentFile->name + "\",\"status\":\"" + status + "\"}"; // 确认消息负载

        // 使用单例模式的sendMessage方法发送确认消息
        mqtt_client::Instance()->sendMessage(FILE_ACK_TOPIC, ackPayload);
    }
    catch (const mqtt::exception &exc)
    {
        std::cerr << "发送确认消息失败:" << exc.what() << std::endl;
    }
}

/**
 * @brief 计算文件的MD5校验和
 */
std::string calculateFileMD5(const std::string &filePath)
{
    std::ifstream file(filePath, std::ios::binary);
    if (!file)
    {
        return "";
    }

    MD5_CTX md5Context;
    MD5_Init(&md5Context);

    char buffer[4096];
    while (file.good())
    {
        file.read(buffer, sizeof(buffer));
        MD5_Update(&md5Context, buffer, file.gcount());
    }

    unsigned char digest[MD5_DIGEST_LENGTH];
    MD5_Final(digest, &md5Context);

    char mdString[33];
    for (int i = 0; i < 16; i++)
    {
        sprintf(&mdString[i * 2], "%02x", (unsigned int)digest[i]);
    }

    return std::string(mdString);
}

/**
 * @brief 保存当前接收的文件
 */
void saveCurrentFile()
{
    if (!currentFile)
    {
        return;
    }

    std::string filePath = (fs::path(fileSaveDir) / currentFile->name).string(); // 文件路径
    std::ofstream file(filePath, std::ios::binary);                              // 创建文件

    if (!file)
    {
        std::cerr << "无法创建文件: " << filePath << std::endl;
        sendFileAck(false);
        currentFile = nullptr;
        return;
    }

    // 写入数据块
    for (const auto &chunk : currentFile->data)
    {
        file.write(reinterpret_cast<const char *>(chunk.data()), chunk.size());
    }

    file.close();
    std::cout << "文件已保存: " << filePath << std::endl;

    // 验证MD5
    bool verifySuccess = true;
    if (!currentFile->md5.empty())
    {
        std::string localMd5 = calculateFileMD5(filePath); // 计算本地MD5
        verifySuccess = (localMd5 == currentFile->md5);    // 比较MD5
        if (verifySuccess)
        {
            std::cout << "文件 " << currentFile->name << " 校验成功" << std::endl;
        }
        else
        {
            std::cout << "文件 " << currentFile->name << " 校验失败（本地md5: " << localMd5 << "，期望md5: " << currentFile->md5 << "）" << std::endl;
        }
    }

    // 发送确认消息
    sendFileAck(verifySuccess);

    if (currentFile->is_last)
    {
        std::cout << "\n所有文件接收完成！" << std::endl;
    }

    // 清除当前文件信息
    currentFile = nullptr;
}

/**
 * @brief 处理文件数据块
 */
void processFileChunk(int chunkId, const std::vector<unsigned char> &chunkData)
{
    // 验证块ID
    if (!currentFile || chunkId < 0 || chunkId >= currentFile->chunks)
    {
        return;
    }

    currentFile->data[chunkId] = std::vector<unsigned char>(chunkData.begin(), chunkData.end());
    currentFile->received_chunks++;

    if (currentFile->received_chunks == currentFile->chunks)
    {
        std::cout << std::endl;
        saveCurrentFile(); // 保存文件
    }
}

/**
 * @brief 处理文件传输消息
 */
void processFileTransferMessage(const std::string &topic, const std::vector<unsigned char> &payload)
{
    std::lock_guard<std::mutex> lock(fileMutex);

    // 解析主题路径
    std::vector<std::string> topicParts;
    std::string topicCopy = topic;
    size_t pos = 0;
    while ((pos = topicCopy.find('/')) != std::string::npos) // 找到第一个'/', 并将其后面的内容作为一个字符串切片
    {
        topicParts.push_back(topicCopy.substr(0, pos));
        topicCopy.erase(0, pos + 1);
    }
    topicParts.push_back(topicCopy);

    // 处理元数据消息
    if (topic == FILE_TRANSFER_META_TOPIC)
    {
        processFileMetadata(std::string(payload.begin(), payload.end())); // 处理元数据
        return;
    }

    // 处理数据块消息
    if (topicParts.size() >= 3 && topicParts[0] == "transferfiles" && topicParts[1] == "data")
    {
        if (!currentFile)
        {
            return;
        }

        try
        {
            int chunkId = std::stoi(topicParts[2]); // 块ID
            processFileChunk(chunkId, payload);     // 处理数据块
        }
        catch (const std::exception &e)
        {
            std::cerr << "处理文件块失败: " << e.what() << std::endl;
        }
    }
}