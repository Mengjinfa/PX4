#include "logger.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <spdlog/details/null_mutex.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

class file_sink : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit file_sink(const std::string &log_filename = "./logs/logger_output.txt")
        : device_path_(log_filename), fout_(log_filename, std::ios_base::app)
    {
        // 1. 确保目录存在
        std::string dir = std::filesystem::path(log_filename).parent_path().string();
        if (!dir.empty() && !std::filesystem::exists(dir))
        {
            std::filesystem::create_directories(dir);
        }
        // 2. 打开文件（追加模式）
        fout_.open(log_filename, std::ios_base::app);

        // 3. 检查是否成功
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

            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            console_sink->set_level(spdlog::level::trace);
            console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

            auto file_sink_instance = std::make_shared<file_sink>();
            file_sink_instance->set_level(spdlog::level::trace);

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