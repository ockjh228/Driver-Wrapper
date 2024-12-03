/**
 * @file ros1_node.hpp
 * @author kyungpyo kim (kyungpyo.kim@acelab.ai)
 * @author wonteak lim (wonteak.lim@acelab.ai)
 * @brief
 * @version 1.0 (2021-11-14) modified by kyungpyo kim, based on node.hpp by
 * wonteak
 * @date 2022-11-14
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_ROS1_NODE_HPP_  // NOLINT
#define BASE_ROS1_NODE_HPP_  // NOLINT

#include <ros/ros.h>
#include <spdlog/spdlog.h>

#include <any>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include "base/ros1_message_handler.hpp"

namespace acelab {
namespace ROS1 {

/**
 * @brief ROS1 Node class
 *
 * Implements ROS1 node and message handler using Singleton pattern
 * Instance can be accessed using getInstance() method
 *
 */
class Node {
private:
  Node() = default;
  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;
  ~Node(){
    for (auto &p : pub_handlers_) {
      p.second.reset();
    }
    for (auto &s : sub_handlers_) {
      s.second.reset();
    }
    logger_->info("ROS1::Node is destroyed");
  }

public:
  static Node &getInstance() {
    static Node instance;
    return instance;
  }

  void init(std::shared_ptr<spdlog::logger> logger, int argc, char **argv) {
    if (!ros::isInitialized()) {
      ros::init(argc, argv, "my_node");
      nh_ = std::make_shared<ros::NodeHandle>();
      logger_ = logger;
      logger_->trace("ROS1::Node::init() {}", ros::this_node::getName());
    }
  }

public:
  /**
   * @brief getting a node name
   * @see getName()
   */
  std::string getName() { return ros::this_node::getName(); }

  /**
   * @brief register a publisher
   * @see registerPublisher()
   * @param name a topic name
   * @param buffer buffer size
   */
  template <class M>
  void registerPublisher(const std::string &name, const uint32_t &buffer) {
    auto p = std::make_shared<PublishHandler<M>>(name, buffer);
    pub_handlers_.insert({name, (std::any)p});
  }

  /**
   * @brief register a subscriber
   * @see registerSubscriber()
   * @param name a topic name
   * @param buffer buffer size
   */
  template <class M>
  void registerSubscriber(const std::string &name, const uint32_t &buffer) {
    auto p = std::make_shared<SubscribeHandler<M>>(name, buffer);
    sub_handlers_.insert({name, (std::any)p});
  }

  /**
   * @brief register a subscriber
   * @see registerSubscriber()
   * @param name a topic name
   * @param buffer buffer size
   * @param f a callback function
   */
  template <class M>
  void registerSubscriber(const std::string &name,
                          const uint32_t &buffer,
                          void (*f)(const M &)) {
    auto p = std::make_shared<SubscribeHandler<M>>(name, buffer);
    p->setCallbackFunc(f);
    sub_handlers_.insert({name, (std::any)p});
  }

  /**
   * @brief register a subscriber
   * @see registerSubscriber()
   * @param name a topic name
   * @param buffer buffer size
   * @param f a callback function
   * @param obj a object pointer
   */
  template <class M, class T>
  void registerSubscriber(const std::string &name,
                          const uint32_t &buffer,
                          void (T::*f)(const M &),
                          T *obj) {
    auto p = std::make_shared<SubscribeHandler<M>>(name, buffer);
    p->setCallbackFunc(std::bind(f, obj, std::placeholders::_1));
    sub_handlers_.insert({name, (std::any)p});
  }

  /**
   * @brief unregister a pubscriber
   * @see unregisterPublish()
   * @param name a topic name
   */
  template <class M>
  void unregisterPublisher(const std::string &name) {
    auto handler = pub_handlers_.find(name);
    if (handler != pub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<PublishHandler<M>>>(handler->second);
      cast_handler->unregister();
      pub_handlers_.erase(handler);
      return;
    }
    logger_->error("unregister publisher failed: {}", name);
  }

  /**
   * @brief unregister a subscriber
   * @see unregisterSubscribe()
   * @param name a topic name
   */
  template <class M>
  void unregisterSubscriber(const std::string &name) {
    auto handler = sub_handlers_.find(name);
    if (handler != sub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<SubscribeHandler<M>>>(handler->second);
      cast_handler->unregister();
      sub_handlers_.erase(handler);
      return;
    }
    logger_->error("unregister subscriber failed: {}", name);
  }

  /**
   * @brief setting a publish message
   * @see setPublishMessage()
   * @param name a topic name
   * @param msg a message
   */
  template <class M>
  void setPublishMessage(const std::string &name, const M &msg) {
    auto handler = pub_handlers_.find(name);
    if (handler != pub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<PublishHandler<M>>>(handler->second);
      cast_handler->setMessage(msg);
      return;
    }
    logger_->error("set publish message failed: {}", name);
  }

  /**
   * @brief getting a subscribe message
   * @see getSubscribeMessage()
   * @param name a topic name
   */
  template <class M>
  std::optional<M> getSubscribeMessage(const std::string &name) {
    auto handler = sub_handlers_.find(name);
    if (handler != sub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<SubscribeHandler<M>>>(handler->second);
      return cast_handler->getMessage();
    }
    logger_->error("get subscribe message failed: {}", name);
    return std::nullopt;
  }

  /**
   * @brief resetting a subscribe message
   * @see resetSubscribeMessage()
   * @param name a topic name
   */
  template <class M>
  void resetSubscribeMessage(const std::string &name) {
    auto handler = sub_handlers_.find(name);
    if (handler != sub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<SubscribeHandler<M>>>(handler->second);
      cast_handler->resetMessage();
    } else {
      logger_->error("reset subscribe message failed: {}", name);
    }
  }

  /**
   * @brief getting a ros paramter
   * @see getParam()
   * @param name a parameter name
   * @param value the returned paramter
   */
  template <class M>
  bool getParam(const std::string &key, const M &value) const {
    return nh_->getParam(key, value);
  }

  /**
   * @brief publish a message
   * @see publishMessage()
   * @param name a topic name
   */
  template <class M>
  void publishMessage(const std::string &name) {
    auto handler = pub_handlers_.find(name);
    if (handler != pub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<PublishHandler<M>>>(handler->second);
      cast_handler->publish();
      return;
    }
    logger_->error("publish message failed: {} is not found", name);
  }

  /**
   * @brief publish a message
   * @see publishMessage()
   * @param name a topic name
   * @param message a message
   */
  template <class M>
  void publishMessage(const std::string &name, const M &msg) {
    auto handler = pub_handlers_.find(name);
    if (handler != pub_handlers_.end()) {
      auto cast_handler =
        std::any_cast<std::shared_ptr<PublishHandler<M>>>(handler->second);
      cast_handler->setMessage(msg);
      cast_handler->publish();
      return;
    }
    logger_->error("publish message failed: {} is not found", name);
  }

public:
  std::shared_ptr<ros::NodeHandle> getHandle() const { return nh_; }

private:
  std::shared_ptr<ros::NodeHandle> nh_;
  std::map<std::string, std::any> pub_handlers_;
  std::map<std::string, std::any> sub_handlers_;
  std::shared_ptr<spdlog::logger> logger_;
};

}  // namespace ROS1
}  // namespace acelab

#endif  // BASE_ROS1_NODE_HPP_ // NOLINT
