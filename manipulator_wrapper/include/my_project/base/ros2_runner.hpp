
/**
 * @file ros2_runner.hpp
 * @author Minchul Lee (minchul.lee@acelab.ai)
 * @author wonteak lim (wonteak.lim@acelab.ai)
 * @brief
 * @version 1.0 (2021-11-14) modified by kyungpyo kim, based on task.hpp by
 * wonteak
 * @date 2022-11-11
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_ROS2_RUNNER_HPP_  // NOLINT
#define BASE_ROS2_RUNNER_HPP_  // NOLINT

#include <fstream>
#include <memory>
#include <rclcpp/executor_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <string>

#include "base/base_runnable.hpp"
#include "base/ros2_node.hpp"

namespace acelab {
namespace ROS2 {

class Runner {
public:
  Runner(std::shared_ptr<BaseRunnable> runnable,
         int argc,
         char **argv,
         std::string node_name,
         std::unordered_map<std::string, int> domain_ids = {{"mission", 2},
                                                            {"integrated", 0}})
      : runnable_(runnable) {
    Node::getInstance().init(argc, argv, node_name + getCpuUid(), domain_ids);
    nh_list_ = Node::getInstance().getHandlers();
    default_domain_name_ = Node::getInstance().getDefaultDomainName();
  }
  ~Runner() = default;

public:
  virtual void init() = 0;
  virtual void start() = 0;
  virtual void end() = 0;
  virtual void diagnose() = 0;

  std::string getCpuUid() {
    std::string res = "";

    std::ifstream infile("/proc/cpuinfo");
    std::string header = "Serial";

    std::string line;
    while (std::getline(infile, line)) {
      std::size_t found = line.find(header);
      if (found != std::string::npos) {
        std::istringstream iss(line);
        std::string buffer;
        std::vector<std::string> result;

        while (std::getline(iss, buffer, ':')) {
          result.push_back(buffer);
        }

        if (result.size() >= 2) {
          res = result.at(1);
          res.replace(res.find(" "), 1, "");
          return res;
        }
      }
    }

    return res;
  }

public:
  void process() {
    // clang-format off
    auto period = runnable_->getParameterManager()->get<double>({"system", "task", "period"}); // NOLINT
    auto num_spin_thread = runnable_->getParameterManager()->get<size_t>({"system", "task", "num_spin_thread"}); // NOLINT
    // clang-format on

    this->init();       // ros runner init
    runnable_->init();  // wrapper init

    startExecutor();
    loop(period);
  }

  double getElapsedSec(const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);

    return it != nh_list_.end()
             ? rclcpp::Duration(nh_list_[find_domain]->get_clock()->now() -
                                process_start_time_)
                 .seconds()
             : 0.;
  }

  std::shared_ptr<rclcpp::Node> getNodeHandler(
    const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    return it != nh_list_.end() ? nh_list_[find_domain] : nullptr;
  };

  void startExecutor() {
    auto num_spin_thread = runnable_->getParameterManager()->get<size_t>(
      {"system", "task", "num_spin_thread"});  // NOLINT
    //*	number of threads to have in the thread pool, the default 0 will use the
    // number of cpu cores found instead
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
      rclcpp::ExecutorOptions(), num_spin_thread);
    for (auto &nh : nh_list_) {
      executor_->add_node(nh.second);
    }

    std::thread spin(std::bind(&Runner::spin, this));
    spin.detach();
  }

  void resumeExcutor(const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);

    if (it != nh_list_.end()) {
      executor_->add_node(nh_list_[find_domain]);
    }
  }

  void pauseExcutor(const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);

    if (it != nh_list_.end()) {
      executor_->remove_node(nh_list_[find_domain]);
    }
  }

private: 
  /**
   * @brief Loop runnable tasks and check rate using health checker
   *
   * @param period
   */
  void loop(const double &period) {
    if (period <= 0.) {
      processRunnable();
      return;
    }

    rclcpp::GenericRate rate(1. / period);
    while (rclcpp::ok()) {
      if (loop_free_) {
        loop_free_ = false;
        std::thread process(
          std::bind(&Runner::processRunnable, this, "default"));
        process.detach();
      } else {
      }
      rate.sleep();
    }

    while (!loop_free_) {
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  void spin() { executor_->spin(); }

  void processRunnable(const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    if (it != nh_list_.end()) {
      process_start_time_ = nh_list_[find_domain]->get_clock()->now();
    }

    this->start();
    runnable_->start();
    runnable_->run();
    runnable_->end();
    this->end();

    runnable_->diagnose();
    this->diagnose();

    loop_free_ = true;
  }

private:
  std::shared_ptr<BaseRunnable> runnable_;

private:
  std::atomic_bool loop_free_{true};

private:
  std::string default_domain_name_ = "default";
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Node>> nh_list_;
  rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
  rclcpp::Time process_start_time_;
};

}  // namespace ROS2
}  // namespace acelab

#endif  // BASE_ROS2_RUNNER_HPP_  // NOLINT