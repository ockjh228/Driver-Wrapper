/**
 * @file ros1_runner.hpp
 * @author kyungpyo kim (kyungpyo.kim@acelab.ai)
 * @author wonteak lim (wonteak.lim@acelab.ai)
 * @brief
 * @version 1.0 (2021-11-14) modified by kyungpyo kim, based on task.hpp by
 * wonteak
 * @date 2022-11-11
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_ROS1_RUNNER_HPP_  // NOLINT
#define BASE_ROS1_RUNNER_HPP_  // NOLINT

#include <ros/ros.h>
#include <spdlog/spdlog.h>

#include <memory>

// clang-format off
#include <health_checker/health_checker.hpp>  // NOLINT
// clang-format on

#include "base/base_runnable.hpp"
#include "base/ros1_node.hpp"

namespace acelab {
namespace ROS1 {

class Runner {
public:
  Runner(std::shared_ptr<BaseRunnable> runnable, int argc, char **argv)
      : runnable_(runnable) {
    logger_ = runnable_->getLoggerHandler()->getLogger();
    Node::getInstance().init(logger_, argc, argv);
    health_checker_ = std::make_shared<system::HealthChecker>();
    health_checker_->enable();
  }
  ~Runner(){
    logger_->info("ROS1::Runner is destroyed");
  }

public:
  virtual void init() = 0;
  virtual void start() = 0;
  virtual void end() = 0;
  virtual void diagnose() = 0;

public:
  void process() {
    // clang-format off
    auto period = runnable_->getParameterManager()->get<double>({"system", "task", "period"}); // NOLINT
    auto num_spin_thread = runnable_->getParameterManager()->get<size_t>({"system", "task", "num_spin_thread"}); // NOLINT
    // clang-format on

    // initailize core module and then initialize ros1 module
    runnable_->init();
    this->init();

    std::thread spin(std::bind(&Runner::spin, this, num_spin_thread));
    spin.detach();

    loop(period);
  }

  double getElapsedSec() {
    return ros::Duration(ros::Time::now() - process_start_time_).toSec();
  }

private:
  /**
   * @brief Loop runnable tasks and check rate using health checker
   *
   * @param period
   */
  void loop(const double &period) {
    logger_->info("ROS1::Runner::loop() is started, period: {}", period);

    if (period <= 0.) {
      logger_->warn("period({}) is not valid, module run once", period);
      processRunnable();
      return;
    }

    ros::Rate rate(1. / period);
    static int skip_count = 0;
    while (ros::ok()) {
      if (loop_free_) {
        loop_free_ = false;
        health_checker_->checkRate(period);
        std::thread process(std::bind(&Runner::processRunnable, this));
        process.detach();
        skip_count = 0;
      } else {
        skip_count++;
        logger_->error("{} is not terminated within the period time, skip {} loop(s)",
                       Node::getInstance().getName(), skip_count);
      }
      rate.sleep();
    }

    while (!loop_free_) {
      logger_->trace("waiting for {} to terminate",
                     Node::getInstance().getName());
      ros::Duration(0.1).sleep();
    }
  }

  void spin(const size_t &num_spin_thread) {
    logger_->info("num_spin_thread: {}", num_spin_thread);
    ros::AsyncSpinner spinner(num_spin_thread);
    spinner.start();
    ros::waitForShutdown();
    logger_->info("ROS1::Runner::spin() end");
  }

  void processRunnable() {
    process_start_time_ = ros::Time::now();
    logger_->trace("ROS1::Runner::processRunnable() start, current time: {}",
                   process_start_time_.toSec());

    if (runnable_->getParameterManager()->update()) {
      logger_->trace("ROS1::Runner::processRunnable() parameter is updated");
    }

    this->start();
    runnable_->start();
    runnable_->run();
    runnable_->end();
    this->end();

    runnable_->diagnose();
    this->diagnose();

    logger_->trace("ROS1::Runner::processRunnable() end, elapsed time: {}",
                   getElapsedSec());
    loop_free_ = true;
  }

public:
  std::shared_ptr<system::HealthChecker> getHealthChecker() {
    return health_checker_;
  }

private:
  std::shared_ptr<BaseRunnable> runnable_;
  std::shared_ptr<spdlog::logger> logger_;
  std::shared_ptr<system::HealthChecker> health_checker_;

private:
  std::atomic_bool loop_free_{true};

private:
  ros::Time process_start_time_;
};

}  // namespace ROS1
}  // namespace acelab

#endif  // BASE_ROS1_RUNNER_HPP_  // NOLINT
