// logger.hpp
#pragma once

#include <memory>
#include <string>
#include <spdlog/logger.h>
#include <spdlog/sinks/base_sink.h>

namespace logging {

// 获取全局日志记录器
std::shared_ptr<spdlog::logger> get_logger();

// 设置全局日志级别
void set_log_level(spdlog::level::level_enum level);

namespace detail {

// 内部用于存储全局日志记录器实例
std::shared_ptr<spdlog::logger>& global_logger();

} // namespace detail

} // namespace logging