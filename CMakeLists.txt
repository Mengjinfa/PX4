# 最低 CMake 版本要求
cmake_minimum_required(VERSION 3.14)

# 项目名称及语言设置
project(PX4 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# 查找其他依赖库（优先使用 CONFIG 模式）
find_package(fmt REQUIRED CONFIG)     
find_package(spdlog REQUIRED CONFIG) 
find_package(PahoMqttCpp REQUIRED CONFIG)
find_package(Threads REQUIRED)
find_package(MAVSDK REQUIRED CONFIG)
find_package(OpenCV REQUIRED) # 可选：只包含需要的模块
find_package(nlohmann_json REQUIRED)
find_package(pugixml REQUIRED)
find_package(apriltag QUIET)  # AprilTag 可能需要手动构建并配置路径
find_package(gazebo REQUIRED)

# 添加可执行目标
add_executable(${PROJECT_NAME}
    src/Global.cpp
    src/async_mqtt.cpp
    src/takeoff_and_land.cpp
    src/vision_guided_mission_flying.cpp
    src/sim_camera_module.cpp
    src/rc.cpp
    src/target_tracker.cpp
    src/drone_controller.cpp
    src/telemetry_monitor.cpp
    src/video_display.cpp
    src/april_tag_detector.cpp
    src/detection_state_machine.cpp
    src/logger.cpp
    src/BeiDouModule.cpp
)

# 包含目录（本地头文件目录）
target_include_directories(${PROJECT_NAME}
    PRIVATE
    ${PAHO_MQTT_CPP_INCLUDE_DIRS}
    ${spdlog_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${GAZEBO_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

link_directories(${GAZEBO_LIBRARY_DIRS})
add_definitions(${GAZEBO_CXX_FLAGS})

# 编译宏定义
target_compile_definitions(${PROJECT_NAME}
    PRIVATE
    SIMULATION
)

# 编译器选项
if(NOT MSVC)
    target_compile_options(${PROJECT_NAME}
        PRIVATE
        -Wall -Wextra -Wpedantic
    )
else()
    target_compile_options(${PROJECT_NAME}
        PRIVATE
        /W3
    )
endif()

# 链接库（使用嵌入的 spdlog 和 fmt）
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

# 如果 AprilTag 是系统安装的，可以这样链接
if(apriltag_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE apriltag::apriltag)
endif()
