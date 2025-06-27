cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0167 NEW)

# 项目配置
project(px4 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成编译命令数据库，用于 IDE 自动补全

# 查找依赖
find_package(fmt REQUIRED CONFIG)
find_package(spdlog REQUIRED CONFIG)
find_package(PahoMqttCpp REQUIRED CONFIG)
find_package(Threads REQUIRED)
find_package(MAVSDK REQUIRED CONFIG)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(pugixml REQUIRED)
find_package(apriltag QUIET)
find_package(gazebo REQUIRED)

# 输出关键依赖版本信息
message(STATUS "Gazebo version: ${gazebo_VERSION}")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")

# 添加可执行文件
add_executable(${PROJECT_NAME}
    src/apriltag_tracker.cpp
    src/mavsdk_members.cpp
    src/sim_camera_module.cpp
    src/telemetry_monitor.cpp
    src/mqtt_client.cpp
    src/flight_procedure.cpp
    src/pid.cpp
    src/landing_state_machine.cpp
    src/fly_mission.cpp
    src/user_task.cpp
    src/main.cpp
)

# 设置目标包含目录
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${CMAKE_SOURCE_DIR}/third_party/eigen
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PAHO_MQTT_CPP_INCLUDE_DIRS}
    ${spdlog_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
)

# 添加编译定义
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    SIMULATION  # 仿真编译宏定义
    # REAL_HARDWARE  # 真机编译宏定义（取消注释启用）
)

# 设置编译选项
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
        -Wall -Wextra -Wpedantic -Wreorder  # 启用成员初始化顺序警告
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE
        /W3
    )
endif()

# 设置链接库
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    fmt::fmt
    spdlog::spdlog
    PahoMqttCpp::paho-mqttpp3
    Threads::Threads
    MAVSDK::mavsdk
    nlohmann_json::nlohmann_json
    pugixml
    ${OpenCV_LIBS}
    ${GAZEBO_LIBRARIES}
)

# 添加 Gazebo CXX 标志
target_compile_options(${PROJECT_NAME}
    PRIVATE
    ${GAZEBO_CXX_FLAGS}
)

# 链接 apriltag
if(apriltag_FOUND)
    message(STATUS "Found apriltag library - linking")
    target_link_libraries(${PROJECT_NAME} PRIVATE apriltag::apriltag)
else()
    message(STATUS "apriltag library not found - AprilTag功能将不可用")
endif()

# 安装规则
install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION bin
)

# 测试支持（如果有测试代码）
include(CTest)
enable_testing()
