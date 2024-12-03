/**
 * @file ros_runnable.hpp
 * * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-08
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef INCLUDE_MY_PROJECT_RUNNER_ROS_RUNNER_HPP_
#define INCLUDE_MY_PROJECT_RUNNER_ROS_RUNNER_HPP_

#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

#include <iostream>
#include <memory>
#include <string>

#include "my_project/base/ros1_runner.hpp"
#include "my_project/core/my_app_runnable.hpp"
#include "my_project/runner/ros1/interface.hpp"

namespace acelab {
namespace ROS {
class Runner : public ROS1::Runner, public Interface {
public:
  Runner(std::shared_ptr<MyAppRunnable> runnable, int argc, char **argv);
  ~Runner();

public:
  /**
   * override virtual functions of ROS1::Runner
   * these functions are called by the ROS1::Runner::loop()
   *
   * init() is called before starting task loop
   *
   * in the task loop, start(), end() and diagnose() are called in order
   *
   * start(): set value to runnable_ using ROS interface
   * end(): publish using ROS interface
   * diagnose(): publish using ROS diagnose interface
   */
  void init() override;
  void start() override;
  void end() override;
  void diagnose() override;

private:
  /**
   * custom callback function
   */
  void callback(const std_msgs::Bool &msg);

private:
  std::shared_ptr<MyAppRunnable> runnable_;
  std::shared_ptr<spdlog::logger> logger_;

private:
  std::string pub1_, pub2_;
  std::string sub1_, sub2_;
};

}  // namespace ROS
}  // namespace acelab

#endif  // INCLUDE_MY_PROJECT_RUNNER_ROS_RUNNER_HPP_
