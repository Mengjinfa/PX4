#include "mavsdk_members.hpp"

using namespace mavsdk;

// Mavsdk_members类的构造函数（初始化对象）
// 功能：将各MAVSDK插件实例绑定到Mavsdk_members
Mavsdk_members::Mavsdk_members(MavlinkPassthrough &mp, MissionRaw &mr, Telemetry &t, Offboard &o, Mission &m, Action &a, Camera &c)
    : action(a), mission(m), telemetry(t), offboard(o), camera(c), mavlink_passthrough(mp), mission_raw(mr) {}
