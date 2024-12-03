/**
 * @file my_app_runnable.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-14
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#include "my_project/core/my_app_runnable.hpp"

#define LICENSE_FUNCTIONCODE 1
#define LICENSE_SUBFUNCTIONCODE 0

namespace acelab {

MyAppRunnable::MyAppRunnable(const std::string& module_name,
                             const std::string& param_path,
                             const std::string& log_dir)
    : BaseRunnable(module_name, param_path, log_dir, LICENSE_FUNCTIONCODE, LICENSE_SUBFUNCTIONCODE) {
  logger_ = getLoggerHandler()->getLogger();
  parameter_manager_ = getParameterManager();
  getLoggerHandler()->setConsoleLevel(
    parameter_manager_->get<std::string>({"system", "verbosity"}));
}

void MyAppRunnable::init() {
  /**
   * logger usage examples
   */
  // logger_->trace("MyAppRunnable::init() trace log");
  // logger_->info("MyAppRunnable::init() info log");
  // logger_->warn("MyAppRunnable::init() warn log");
  // logger_->error("MyAppRunnable::init() error log");
  // logger_->debug("MyAppRunnable::init() debug log");
  // logger_->critical("MyAppRunnable::init() critical log");
}

void MyAppRunnable::start() {
  /**
   * get parameters from parameter manager
   * parameter is updated before the task loop is running
   */
  a_ = parameter_manager_->get<double>({"runtime", "a"});
  b_ = parameter_manager_->get<std::string>({"runtime", "b"});
}

void MyAppRunnable::run() {
  // logger_->info("MyAppRunnable::run() data: {}", data_ ? "true" : "false");
  // logger_->info("MyAppRunnable::run() a: {}", a_);
  // logger_->info("MyAppRunnable::run() b: {}", b_);
  // logger_->info("MyAppRunnable::run() c: {}", c_);
}

void MyAppRunnable::end() { logger_->info("MyAppRunnable::end()"); }

void MyAppRunnable::diagnose() { logger_->info("MyAppRunnable::diagnose()"); }

void MyAppRunnable::setData(bool&& data) { data_ = std::move(data); }
void MyAppRunnable::setA(double&& a) { a_ = std::move(a); }
void MyAppRunnable::setB(std::string&& b) { b_ = std::move(b); }
void MyAppRunnable::setC(int64_t&& c) { c_ = std::move(c); }

bool MyAppRunnable::getData() const { return data_; }
double MyAppRunnable::getA() const { return a_; }
std::string MyAppRunnable::getB() const { return b_; }
int64_t MyAppRunnable::getC() const { return c_; }

}  // namespace acelab
