#ifndef BEIDOU_HPP
#define BEIDOU_HPP

#include "mavsdk_members.hpp"
#include "singleton.hpp"
#include <iostream>

#define MAX_NUM 2
struct Position
{
    double longitude;
    double latitude;
};

class CBeidouModule
{
public:
    CBeidouModule();
    ~CBeidouModule();
    void setRawMsg(std::string rawmsg, int index) { _rawmsg[index] = rawmsg; }
    std::string getRawMsg(int index) { return _rawmsg[index]; }
    bool get_position(int index, Position &pos);

private:
    std::string getData4Rawmsg(const std::string &data, int index);
    std::string _rawmsg[MAX_NUM];
};

typedef NormalSingleton<CBeidouModule> BeidouModule;

#endif
