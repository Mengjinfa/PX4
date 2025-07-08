#ifndef FILE_TRANSFER_HPP
#define FILE_TRANSFER_HPP

#include <fstream>
#include <memory>
#include <mqtt/client.h>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <memory>
#include <nlohmann/json.hpp> // 添加 JSON 库包含
#include <string>
#include <vector>

using json = nlohmann::json; // 定义 json 类型别名

// 主题定义
const std::string FILE_TRANSFER_META_TOPIC = "transferfiles/meta";
const std::string FILE_TRANSFER_DATA_TOPIC = "transferfiles/data/#";
const std::string FILE_ACK_TOPIC = "transferfiles/ack";

// 文件元数据结构
struct FileMetadata
{
    std::string name;
    size_t size;
    int chunks;
    int received_chunks;
    std::vector<std::vector<unsigned char>> data;
    bool is_last;
    std::string md5;
};

void processFileTransferMessage(const std::string &topic, const std::vector<unsigned char> &payload); // 处理文件传输消息（元数据或数据块）
void setFileSaveDirectory(const std::string &dir);                                                    // 设置文件保存目录（自动创建目录）

#endif // FILE_TRANSFER_HPP