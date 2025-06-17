// logger.cpp
#include "logger.hpp"
#include <filesystem> // 添加 filesystem 头文件
#include <fstream>
#include <iostream>
#include <mutex>
#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

// 自定义文件日志 sink
class file_sink : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit file_sink(const std::string &log_filename = "./logs/logger_output.txt") : device_path_(log_filename), fout_(log_filename, std::ios_base::app)
    {

        // 创建 logs 目录（如果不存在）
        std::string dir = "./logs";
        if (!std::filesystem::exists(dir))
        {
            std::filesystem::create_directory(dir);
        }

        if (!fout_.is_open())
        {
            std::cerr << "Failed to open log file: " << device_path_ << std::endl;
        }
    }

protected:
    void sink_it_(const spdlog::details::log_msg &msg) override
    {
        // 将日志消息转换为字符串
        spdlog::memory_buf_t formatted;
        formatter_->format(msg, formatted);

        if (fout_.is_open())
        {
            fout_ << fmt::to_string(formatted);
            fout_.flush(); // 确保立即写入
        }
        else
        {
            // 可选：记录错误日志，使用全局 logger
            auto logger = logging::get_logger();
            if (logger)
            {
                logger->error("File write failed: {}", device_path_);
            }
        }
    }

    void flush_() override
    {
        if (fout_.is_open())
        {
            fout_.flush();
        }
    }

private:
    std::string device_path_;
    std::ofstream fout_;
};

namespace logging
{

    namespace detail
    {
        std::shared_ptr<spdlog::logger> &global_logger()
        {
            static std::shared_ptr<spdlog::logger> instance;
            return instance;
        }
    }

    std::shared_ptr<spdlog::logger> get_logger()
    {
        auto &logger = detail::global_logger();
        if (!logger)
        {
            // 控制台日志器
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_level(spdlog::level::trace);
            console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

            // 文件日志器
            auto file_sink_instance = std::make_shared<file_sink>(); // 默认写入 ./logs/logger_output.txt
            file_sink_instance->set_level(spdlog::level::trace);

            // 创建 logger，并添加多个 sink
            logger = std::make_shared<spdlog::logger>("global", spdlog::sinks_init_list{console_sink, file_sink_instance});
            logger->set_level(spdlog::level::trace);
            logger->info("Global logger initialized");
        }
        return logger;
    }

    void set_log_level(spdlog::level::level_enum level)
    {
        auto logger = get_logger();
        logger->set_level(level);
    }

} // namespace logging