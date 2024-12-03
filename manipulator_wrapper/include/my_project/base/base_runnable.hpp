/**
 * @file base_runnable.hpp
 * @author kyungpyo kim (kyungpyo.kim@acelab.ai)
 * @brief
 * @version 0.1
 * @date 2022-11-11
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_RUNNABLE_HPP_  // NOLINT
#define BASE_RUNNABLE_HPP_  // NOLINT

#include <memory>                                // NOLINT
#include <string>                                // NOLINT

#include "base/logger.hpp"
#include "base/parameter_manager.hpp"

namespace acelab {
class BaseRunnable {
public:
  BaseRunnable(const std::string &name,
               const std::string &param_path,
               const std::string &log_dir,
               const uint32_t function_code = 0,
               const uint32_t sub_function_code = 0,
               const size_t &log_num = 1,
               const size_t &log_size = 2) {
    // clang-format off
    logger_handler_ = std::make_shared<LoggerHandler>(name, log_dir, log_num, log_size); // NOLINT
    parameter_manager_ = std::make_shared<ParameterManager>(param_path, logger_handler_->getLogger()); // NOLINT
    // clang-format on
  }
  virtual ~BaseRunnable() = default;

public:
  virtual void init() = 0;
  virtual void start() = 0;
  virtual void run() = 0;
  virtual void end() = 0;
  virtual void diagnose() = 0;

public:
  std::shared_ptr<ParameterManager> getParameterManager() {
    return parameter_manager_;
  }
  std::shared_ptr<LoggerHandler> getLoggerHandler() { return logger_handler_; }

private:
  std::shared_ptr<ParameterManager> parameter_manager_;
  std::shared_ptr<LoggerHandler> logger_handler_;
};

}  // namespace acelab

#endif  // BASE_RUNNABLE_HPP_ // NOLINT
