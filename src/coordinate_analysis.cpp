#include "coordinate_analysis.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <string>

// 北斗数据全局实例
BeiDouData beidou_data;

// 解析DDMM.MMMMM格式为十进制度
double convertDMMToDD(const std::string &dmmStr)
{
    // 字符串转double
    try
    {
        double dmmValue = std::stod(dmmStr);

        // 分离度和分
        int degrees = static_cast<int>(dmmValue) / 100;
        double minutes = dmmValue - (degrees * 100);

        // 验证分钟部分是否有效
        if (minutes < 0 || minutes >= 60)
        {
            throw std::invalid_argument("无效的分钟值，必须在0-59.9999范围内");
        }

        // 转换为十进制度数
        return degrees + minutes / 60.0;
    }
    catch (const std::exception &e)
    {
        throw std::invalid_argument("经纬度转换失败: " + std::string(e.what()));
    }
}

// 计算NMEA校验和
unsigned char calculateChecksum(const std::string &message)
{
    // 查找消息起始符 $
    size_t startPos = message.find('$');
    if (startPos == std::string::npos)
    {
        throw std::invalid_argument("无效的NMEA消息: 找不到起始符$");
    }

    // 查找校验和分隔符 *
    size_t starPos = message.find('*');
    if (starPos == std::string::npos)
    {
        throw std::invalid_argument("无效的NMEA消息: 找不到校验和分隔符*");
    }

    // 只计算 $ 和 * 之间的字符
    unsigned char checksum = 0;
    for (size_t i = startPos + 1; i < starPos; i++)
    {
        checksum ^= message[i];
    }

    return checksum;
}

// 安全地从字符串转换为数值类型
template <typename T>
std::optional<T> safeConvert(const std::string &str)
{
    if (str.empty())
    {
        return std::nullopt;
    }

    try
    {
        if constexpr (std::is_same_v<T, int>)
        {
            return std::stoi(str);
        }
        else if constexpr (std::is_same_v<T, double>)
        {
            return std::stod(str);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "转换错误: " << e.what() << ", 值: " << str << std::endl;
    }

    return std::nullopt;
}

// 解析BDGGA格式的NMEA消息
bool analyzeBeidouData(const std::string &nmea, BeiDouData &data)
{
    // 检查消息格式
    if (nmea.substr(0, 6) != "$BDGGA")
    {
        std::cerr << "错误：非BDGGA格式消息" << std::endl;
        return false;
    }

    // 查找校验和分隔符
    size_t starPos = nmea.find('*');
    std::string msgBody = starPos != std::string::npos ? nmea.substr(0, starPos) : nmea;

    // 分割字段
    std::vector<std::string> fields;
    std::stringstream ss(msgBody);
    std::string field;

    while (std::getline(ss, field, ',')) // 逐个分割字段
    {
        fields.push_back(field); // 保存字段
    }

    // 验证字段数量
    if (fields.size() < 14)
    {
        std::cerr << "错误：BDGGA字段不足，需要至少14个字段，实际" << fields.size() << "个" << std::endl;
        return false;
    }

    // 解析纬度
    data.latitudeStr = fields[2];  // 保存原始字符串
    if (!data.latitudeStr.empty()) // 非空
    {
        data.latitude = convertDMMToDD(data.latitudeStr); // 十进制
    }
    else
    {
        data.latitude = 0.0;
    }

    // 解析经度
    data.longitudeStr = fields[4];  // 保存原始字符串
    if (!data.longitudeStr.empty()) // 非空
    {
        data.longitude = convertDMMToDD(data.longitudeStr); // 十进制
    }
    else
    {
        data.longitude = 0.0;
    }

    // 解析其他字段
    data.fixStatus = safeConvert<int>(fields[6]).value_or(0); // 定位状态
    data.satNum = safeConvert<int>(fields[7]).value_or(0);    // 卫星数

    // 校验和验证
    if (starPos != std::string::npos && starPos + 3 <= nmea.length())
    {
        std::string checksumStr = nmea.substr(starPos + 1, 2);

        // 解析十六进制字符串
        unsigned int msgChecksum = 0;
        std::istringstream iss(checksumStr);
        iss >> std::hex >> msgChecksum;

        if (!iss.fail() && msgChecksum <= 0xFF)
        {
            unsigned char calcChecksum = calculateChecksum(nmea);
            if (calcChecksum != static_cast<unsigned char>(msgChecksum))
            {
                std::cerr << "警告：BDGGA校验和错误，计算值: 0x"
                          << std::hex << static_cast<int>(calcChecksum)
                          << "，消息值: 0x" << checksumStr << std::endl;
            }
        }
        else
        {
            std::cerr << "警告：无法解析校验和值: " << checksumStr << std::endl;
        }
    }

    return true;
}

// beidou_A话题处理函数
void handleBeiDouMessage(const std::vector<unsigned char> &payload)
{
    try
    {
        // 将二进制负载转换为字符串
        std::string payloadStr(payload.begin(), payload.end());

        if (analyzeBeidouData(payloadStr, beidou_data))
        {
            // 输出解析结果
            std::cout << "=== 收到北斗BDGGA数据 ===" << std::endl;
            std::cout << std::fixed << std::setprecision(10);
            std::cout << "纬度: " << beidou_data.latitude << "°" << std::endl;
            std::cout << "经度: " << beidou_data.longitude << "°" << std::endl;
            std::cout << "纬度: " << beidou_data.latitudeStr << "°" << std::endl;
            std::cout << "经度: " << beidou_data.longitudeStr << "°" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "处理BDGGA消息时发生异常: " << e.what() << std::endl;
    }
}