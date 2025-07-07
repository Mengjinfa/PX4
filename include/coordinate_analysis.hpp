#ifndef COORDINATE_ANALYSIS_HPP
#define COORDINATE_ANALYSIS_HPP

#include "mavsdk_members.hpp"

// 北斗数据结构体
struct BeiDouData
{
    double latitude;          // 纬度
    double longitude;         // 经度
    std::string latitudeStr;  // 纬度字符串
    std::string longitudeStr; // 经度字符串
    int fixStatus;            // 定位状态
    int satNum;               // 可用卫星数
};
extern BeiDouData beidou_data;

void handleBeiDouMessage(const std::vector<unsigned char> &payload); // 北斗消息处理函数

#endif // COORDINATE_ANALYSIS_HPP