
cmake_minimum_required(VERSION 3.14)
cmake_policy(SET CMP0167 NEW)

project(px4 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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


include_directories(${CMAKE_SOURCE_DIR}/third_party/eigen)


add_executable(${PROJECT_NAME}
    src/async_mqtt.cpp
    src/sim_camera_module.cpp
    src/telemetry_monitor.cpp
    src/apriltag_tracker.cpp
    src/detection_state_machine.cpp
    src/logger.cpp
    src/BeiDouModule.cpp
    src/mqtt_client.cpp
    src/flight_procedure.cpp
    src/pid.cpp
    src/state_machine.cpp
    src/main.cpp
)


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

# target_compile_definitions(${PROJECT_NAME}
#     PRIVATE
#     REAL_HARDWARE
# )

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


if(apriltag_FOUND)
    target_link_libraries(${PROJECT_NAME} PRIVATE apriltag::apriltag)
endif()