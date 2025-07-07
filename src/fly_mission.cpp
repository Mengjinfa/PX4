#include "fly_mission.hpp"
#include "coordinate_analysis.hpp"

#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <chrono>            // 用于计时
#include <fstream>           // 用于读写文件
#include <functional>        // 用于定义函数对象
#include <future>            // 用于异步调用
#include <iostream>          // 用于输出
#include <nlohmann/json.hpp> // 用于解析JSON文件
#include <thread>            // 用于多线程
#include <vector>            //一种序列容器，它允许你在运行时动态地插入和删除元素。

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

/**
 * 创建一个任务项（航点）
 *
 * @param latitude_deg 纬度（度）
 * @param longitude_deg 经度（度）
 * @param relative_altitude_m 相对起飞点高度（米）
 * @param speed_m_s 飞行速度（米/秒）
 * @param yaw_deg 航向角（度）
 * @param is_fly_through 是否直接飞过（不停留）
 * @return 配置好的任务项
 */
static Mission::MissionItem make_mission_item(
    double latitude_deg,
    double longitude_deg,
    float relative_altitude_m,
    float speed_m_s,
    float yaw_deg,
    bool is_fly_through)
{
    Mission::MissionItem waypoint{};

    waypoint.latitude_deg = latitude_deg;
    waypoint.longitude_deg = longitude_deg;
    waypoint.relative_altitude_m = relative_altitude_m;
    waypoint.speed_m_s = speed_m_s;
    waypoint.yaw_deg = yaw_deg;
    waypoint.is_fly_through = is_fly_through;

    return waypoint;
}

/**
 * @beif:从QGroundControl导出的.plan文件读取任务航点
 *
 *  该函数解析QGroundControl导出的标准.plan格式JSON文件，提取其中的航点信息。

 * @param filename .plan文件路径
 * @return 包含任务航点的列表，解析失败时返回空列表
 *
 * 解析流程：
 * 1. 打开并读取JSON文件内容
 * 2. 验证文件结构是否符合QGC .plan格式
 * 3. 遍历所有任务项，提取有效航点信息
 * 4. 转换为MAVSDK支持的MissionItem格式
 *
 * 文件格式要求：
 * {
 *   "mission": {
 *     "items": [
 *       {
 *         "autoContinue": true, // 是否自动继续到下一个航点
 *         "command": 16,        // 任务命令字段，16=航点，22=起飞
 *         "frame": 3,           // 坐标系
 *         "params": [ ... ],    // 参数数组，包含经纬度和高度等信息
 *       },
 *       ...
 *     ]
 *   }
 * }
 */
static std::vector<mavsdk::Mission::MissionItem> read_qgroundcontrol_plan(const std::string &filename)
{
    std::vector<mavsdk::Mission::MissionItem> mission_items;
    float current_speed = 1.0f; // 默认速度，仅在没有178命令时使用

    try
    {
        // 打开指定的.plan文件
        std::ifstream file(filename);
        if (!file.is_open()) // 文件不存在或不可访问，返回空列表
        {
            std::cerr << "无法打开文件: " << filename << std::endl;
            return mission_items;
        }

        // 解析JSON内容
        nlohmann::json json_data;
        file >> json_data;
        file.close();

        // 验证文件结构是否符合QGC .plan格式
        if (!json_data.contains("mission") ||
            !json_data["mission"].contains("items") ||
            !json_data["mission"]["items"].is_array())
        {
            std::cerr << "文件格式错误: 不是有效的QGC .plan文件" << std::endl;
            return mission_items;
        }

        // 遍历所有任务项，提取航点信息
        for (const auto &item : json_data["mission"]["items"])
        {
            // 检查必要字段是否存在
            if (!item.contains("autoContinue") ||
                !item.contains("command") ||
                !item.contains("frame") ||
                !item.contains("params"))
            {
                continue; // 跳过不完整的任务项
            }

            // 获取命令类型和坐标系
            int command = item["command"].get<int>();
            int frame = item["frame"].get<int>();

            // 处理速度设置命令 (MAV_CMD_DO_CHANGE_SPEED = 178)
            if (command == 178)
            {
                const auto &params = item["params"];
                if (params.size() > 1)
                {
                    // params[0]: 速度类型 (0=空速, 1=地速)
                    // params[1]: 速度值 (m/s)
                    current_speed = params[1].get<float>();

                    std::cout << "速度设置为: " << current_speed << " m/s" << std::endl;
                }
                continue; // 速度命令不生成航点
            }

            // 处理航点命令 (MAV_CMD_NAV_TAKEOFF = 22, MAV_CMD_NAV_WAYPOINT = 16)
            if (frame == 3 && (command == 16 || command == 22))
            {
                const auto &params = item["params"];

                float yaw = params[3].get<float>();
                double latitude = params[4].get<double>();
                double longitude = params[5].get<double>();
                float altitude = params[6].get<float>();
                bool fly_through = item.contains("autoContinue") && item["autoContinue"].get<bool>();

                // 使用 push_back 方法向 vector 中添加元素
                mission_items.push_back(make_mission_item(latitude, longitude, altitude, current_speed, yaw, fly_through));

                std::cout << "已添加航点: 纬度=" << latitude
                          << ", 经度=" << longitude
                          << ", 高度=" << altitude << "m"
                          << ", 速度=" << current_speed << "m/s"
                          << ", 航向=" << yaw << "°"
                          << ", 自动继续=" << (fly_through ? "是" : "否") << std::endl;
            }
        }

        // 检查是否成功提取到航点
        if (mission_items.empty())
        {
            std::cerr << "未找到有效的航点数据" << std::endl;
        }
    }
    catch (const std::exception &e)
    {
        // 捕获并处理JSON解析异常
        std::cerr << "解析文件时出错: " << e.what() << std::endl;
    }

    return mission_items;
}

/**
 * 创建自定义任务航点
 *
 * @return 包含自定义航点的列表
 */
static std::vector<mavsdk::Mission::MissionItem> create_custom_waypoints()
{
    std::vector<Mission::MissionItem> mission_items;

    // 起飞点
    mission_items.push_back(make_mission_item(47.3977508, 8.5456073, 5.0f, 5.0f, 90, false));

    // 航点1
    mission_items.push_back(make_mission_item(beidou_data.latitude, beidou_data.longitude, 10.0f, 15.0f, 90, false));

    std::cout << "已创建" << mission_items.size() << "个自定义航点" << std::endl;
    return mission_items;
}

/**
 * 上传航点任务到无人机
 *
 * @param mission MAVSDK任务插件实例
 * @param mission_items 任务航点列表
 * @return 上传成功返回true，失败返回false
 */
static bool upload_mission(mavsdk::Mission &mission, const std::vector<mavsdk::Mission::MissionItem> &mission_items)
{
    if (mission_items.empty())
    {
        std::cerr << "无法上传空任务" << std::endl;
        return false;
    }

    std::cout << "清除现有任务..." << std::endl;
    mission.clear_mission();

    std::cout << "上传任务到无人机..." << std::endl;
    Mission::MissionPlan mission_plan{};
    mission_plan.mission_items = mission_items;

    const Mission::Result upload_result = mission.upload_mission(mission_plan);

    if (upload_result != Mission::Result::Success)
    {
        std::cerr << "航线任务上传失败: " << upload_result << std::endl;
        return false;
    }

    std::cout << "任务上传成功，共" << mission_items.size() << "个航点" << std::endl;
    return true;
}

/**
 * 执行航点任务并监控状态
 *
 * @param action MAVSDK动作插件实例
 * @param mission MAVSDK任务插件实例
 * @return 任务完成返回true，失败返回false
 */
static bool execute_and_monitor_mission(mavsdk::Action &action, mavsdk::Mission &mission)
{
    // 解锁无人机准备起飞
    std::cout << "准备解锁..." << std::endl;
    const Action::Result arm_result = action.arm();
    if (arm_result != Action::Result::Success)
    {
        std::cerr << "解锁失败: " << arm_result << std::endl;
        return false;
    }
    std::cout << "解锁成功" << std::endl;

    // 订阅任务进度回调函数
    mission.subscribe_mission_progress(
        [](Mission::MissionProgress progress)
        {
            std::cout << "当前航点: " << progress.current << " / " << progress.total << '\n';
        });

    // 开始执行任务
    std::cout << "开始航点任务..." << std::endl;
    Mission::Result start_result = mission.start_mission();
    if (start_result != Mission::Result::Success)
    {
        std::cerr << "启动任务失败: " << start_result << std::endl;
        return false;
    }

    // 等待任务完成，设置超时时间
    const int timeout_seconds = 300; // 5分钟超时
    std::cout << "等待任务完成，超时时间: " << timeout_seconds << "秒..." << std::endl;

    auto start_time = std::chrono::steady_clock::now();

    // 循环检查任务是否完成
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(timeout_seconds))
    {
        // 使用is_mission_finished()同步检查任务状态
        auto [result, is_finished] = mission.is_mission_finished();

        if (result == Mission::Result::Success)
        {
            if (is_finished)
            {
                std::cout << "任务已成功完成" << std::endl;
                return true;
            }
        }
        else
        {
            std::cerr << "检查任务状态失败: " << result << std::endl;
            // 继续检查，不直接退出
        }

        // 短暂休眠避免CPU占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    std::cerr << "任务执行超时，未在指定时间内完成" << std::endl;
    return false;
}

/**
 * 综合函数：根据文件或自定义创建航点，上传并执行任务
 *
 * @param mavsdk MAVSDK成员集合
 * @param plan_file 任务文件路径，为空则使用自定义航点
 * @return 执行成功返回0，失败返回错误码
 */
int fly_mission(Mavsdk_members &mavsdk, const std::string &plan_file)
{
    Action &action = mavsdk.action;
    Mission &mission = mavsdk.mission;

    std::vector<Mission::MissionItem> mission_items;

    // 获取航点数据
    if (!plan_file.empty())
    {
        std::cout << "正在从文件读取任务: " << plan_file << std::endl;
        mission_items = read_qgroundcontrol_plan(plan_file);
        if (mission_items.empty())
        {
            std::cerr << "读取任务文件失败，退出" << std::endl;
            return 0;
        }
    }
    else
    {
        std::cout << "使用默认任务航点..." << std::endl;
        mission_items = create_custom_waypoints();
    }

    // 上传任务
    if (!upload_mission(mavsdk.mission, mission_items))
    {
        return 0;
    }

    // 执行并监控任务
    return execute_and_monitor_mission(mavsdk.action, mavsdk.mission);
}