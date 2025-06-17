#include "BeiDouModule.h"

CBeidouModule::CBeidouModule()
{
    for (int i = 0; i < MAX_NUM; ++i)
        _rawmsg[i] = "";
}

CBeidouModule::~CBeidouModule()
{
}

bool CBeidouModule::get_position(int index, Position &pos)
{
    if (_rawmsg[index] == "")
        return false;
    if (getData4Rawmsg(_rawmsg[0], 2) == "" || getData4Rawmsg(_rawmsg[0], 4) == "")
        return false;
    if (index == 0)
    {
        pos.latitude = std::stod(getData4Rawmsg(_rawmsg[0], 2));
        pos.longitude = std::stod(getData4Rawmsg(_rawmsg[0], 4));
    }
    return true;
}

std::string CBeidouModule::getData4Rawmsg(const std::string &data, int index)
{
    std::vector<std::string> fields;
    size_t start = 0;
    size_t end = data.find(',');

    while (end != std::string::npos)
    {
        fields.push_back(data.substr(start, end - start));
        start = end + 1;
        end = data.find(',', start);
    }
    fields.push_back(data.substr(start)); // 处理最后一个字段

    return fields[index];
}
