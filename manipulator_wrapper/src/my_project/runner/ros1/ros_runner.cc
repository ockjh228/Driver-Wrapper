/**
 * @file ros_runnable.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#include "my_project/runner/ros1/ros_runner.hpp"

#include "my_project/base/base_runnable.hpp"

#define PUB_QUEUE_SIZE 1
#define SUB_QUEUE_SIZE 1

namespace acelab {
namespace ROS {

Runner::Runner(std::shared_ptr<MyAppRunnable> runnable, int argc, char **argv)
    : ROS1::Runner(
        std::dynamic_pointer_cast<BaseRunnable>(runnable), argc, argv),
      runnable_(runnable) {
  logger_ = runnable_->getLoggerHandler()->getLogger();
}

Runner::~Runner() {
  ROS1::Node::getInstance().unregisterPublisher<std_msgs::Bool>(pub1_);
  ROS1::Node::getInstance().unregisterPublisher<std_msgs::Int64>(pub2_);
  ROS1::Node::getInstance().unregisterSubscriber<std_msgs::Bool>(sub1_);
  ROS1::Node::getInstance().unregisterSubscriber<std_msgs::Int64>(sub2_);
}

void Runner::init() {
  logger_->info("ROS Runner init");

  /**
   * register publisher and subscriber
   * they are managed by topic name like pub1_ and sub1_
   */
  // clang-format off
  pub1_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "pub1"}, "default_pub1");
  ROS1::Node::getInstance().registerPublisher<std_msgs::Bool>(pub1_, PUB_QUEUE_SIZE);
  pub2_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "pub2"}, "default_pub2");
  ROS1::Node::getInstance().registerPublisher<std_msgs::Int64>(pub2_, PUB_QUEUE_SIZE);
  sub1_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "sub1"});
  ROS1::Node::getInstance().registerSubscriber<std_msgs::Bool>(sub1_, SUB_QUEUE_SIZE, &Runner::callback, this);
  sub2_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "sub2"});
  ROS1::Node::getInstance().registerSubscriber<std_msgs::Int64>(sub2_, SUB_QUEUE_SIZE);
  // clang-format on
}

void Runner::start() {
  /**
   * get message from topic
   * if message is not received, it returns std::nullopt
   */
  auto sub_msg =
    ROS1::Node::getInstance().getSubscribeMessage<std_msgs::Int64>(sub2_);
  if (sub_msg) {
    runnable_->setC(convert<std_msgs::Int64, int64_t>(*sub_msg));
  }
}

void Runner::end() {
  /**
   * publish message to topic
   */
  ROS1::Node::getInstance().publishMessage<std_msgs::Int64>(
    pub2_, convert<int64_t, std_msgs::Int64>(runnable_->getC()));
}

void Runner::diagnose() { logger_->info("ROS Runner diagnose"); }

void Runner::callback(const std_msgs::Bool &msg) {
  runnable_->setData(std::move(convert<std_msgs::Bool, bool>(msg)));
  ROS1::Node::getInstance().publishMessage<std_msgs::Bool>(
    pub1_, convert<bool, std_msgs::Bool>(runnable_->getData()));
}

}  // namespace ROS
}  // namespace acelab
