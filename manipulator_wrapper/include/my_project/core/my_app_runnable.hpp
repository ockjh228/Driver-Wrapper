/**
 * @file core_runnable.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-08
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef INCLUDE_MY_PROJECT_CORE_MY_APP_RUNNABLE_HPP_
#define INCLUDE_MY_PROJECT_CORE_MY_APP_RUNNABLE_HPP_

#include <spdlog/spdlog.h>

#include <iostream>
#include <memory>
#include <string>

#include "my_project/base/base_runnable.hpp"

namespace acelab {

class MyAppRunnable : public BaseRunnable {
public:
  MyAppRunnable(const std::string &module_name,
                const std::string &param_path,
                const std::string &log_dir);
  ~MyAppRunnable() = default;

private:
  /**
   * override virtual functions of BaseRunnable
   * these functions are called by the Runner
   *
   * init() is called before starting task loop
   *
   * in the task loop, start(), run(), end() and diagnose() are called in order
   *
   * before the start() is called, the Runner::start() is called
   * after the end() is called, the Runner::end() is called
   * after the diagnose() is called, the Runner::diagnose() is called
   */
  void init() override;
  void start() override;
  void run() override;
  void end() override;
  void diagnose() override;

public:
  void setData(bool &&data);
  void setA(double &&a);
  void setB(std::string &&b);
  void setC(int64_t &&c);

public:
  bool getData() const;
  double getA() const;
  std::string getB() const;
  int64_t getC() const;

private:
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<ParameterManager> parameter_manager_;

private:
  std::atomic_bool data_{false};
  double a_{0.};
  std::string b_{""};
  int64_t c_{0};
};

}  // namespace acelab

#endif  // INCLUDE_MY_PROJECT_CORE_MY_APP_RUNNABLE_HPP_
