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
double convertAndScale(const std::string &str)
{
    if (str.empty())
        throw std::invalid_argument("输入字符串为空");

    // 去除前导零
    size_t firstNonZero = str.find_first_not_of('0');
    std::string trimmed;

    if (firstNonZero == std::string::npos)
    {
        // 全零字符串
        trimmed = "0";
    }
    else
    {
        trimmed = str.substr(firstNonZero);

        // 处理小数点在开头的情况
        if (!trimmed.empty() && trimmed[0] == '.')
        {
            trimmed = "0" + trimmed;
        }
    }

    // 转换为浮点数
    double value = 0.0;
    try
    {
        value = std::stod(trimmed); // 将字符串转换为双精度浮点数（double）的函数
    }
    catch (const std::invalid_argument &)
    {
        throw std::invalid_argument("无法转换为有效数字: " + str);
    }
    catch (const std::out_of_range &)
    {
        throw std::out_of_range("数值超出范围: " + str);
    }

    // 小数点右移两位
    return value / 100;
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
    data.latitudeStr = fields[2];                      // 保存原始字符串
    data.latitude = convertAndScale(data.latitudeStr); // 十进制

    // 解析经度
    data.longitudeStr = fields[4];                       // 保存原始字符串
    data.longitude = convertAndScale(data.longitudeStr); // 十进制

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