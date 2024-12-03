/**
 * @file logger.hpp
 * @author kyungpyo kim (kyungpyo.kim@acelab.ai)
 * @brief
 * @version 0.1
 * @date 2022-11-14
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_LOGGER_HPP_  // NOLINT
#define BASE_LOGGER_HPP_  // NOLINT

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string>

#define LOG_PATTERN "[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] [thread %t] %v"

namespace acelab {
class LoggerHandler {
public:
  LoggerHandler(const std::string &log_name,
                const std::string &log_dir,
                const size_t &log_num,
                const size_t &log_size) {
    auto log_file = log_dir + "/" + log_name;
    auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
      log_file, 1024 * 1024 * log_size, log_num);
    file_sink->set_level(spdlog::level::trace);

    console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

    logger_ = std::make_shared<spdlog::logger>(
      log_name, spdlog::sinks_init_list{file_sink, console_sink_});
    logger_->set_level(spdlog::level::trace);
    logger_->set_pattern(LOG_PATTERN);
    logger_->info("log file path: {}", log_file);
    logger_->trace("trace level file logger");
  }
  ~LoggerHandler(){
    logger_->info("LoggerHandler is terminated");
    logger_->flush();
  }

public:
  std::shared_ptr<spdlog::logger> getLogger() { return logger_; }
  void setConsoleLevel(const std::string &level) {
    console_sink_->set_level(spdlog::level::from_str(level));
  }

private:
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink_;
};

}  // namespace acelab

#endif  // BASE_LOGGER_HPP_ // NOLINT
