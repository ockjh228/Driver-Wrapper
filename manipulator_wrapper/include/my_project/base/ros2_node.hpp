
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

#ifndef BASE_ROS2_NODE_HPP_  // NOLINT
#define BASE_ROS2_NODE_HPP_  // NOLINT

#include <any>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "base/ros2_message_handler.hpp"

namespace acelab {
namespace ROS2 {

/**
 * @brief ROS2 Node class
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
  ~Node() = default;

public:
  static Node &getInstance() {
    static Node instance;
    return instance;
  }

  void init(int argc,
            char **argv,
            std::string node_name,
            std::unordered_map<std::string, int> domain_ids) {
    //* ROS multiple domain id configurations
    std::cout << "----------------------------------" << std::endl;
    std::cout << "[Domain ID table]" << std::endl;
    bool is_default = false;
    for (auto &id : domain_ids) {
      if (id.first == "default") {
        auto val = std::getenv("ROS_DOMAIN_ID");
        id.second = val != nullptr ? std::stoi(val) : 0;
        is_default = true;
      }
      std::cout << " - " << id.first << ": " << id.second << std::endl;
      default_domain_name_ = id.first;
    }
    if(is_default){
      default_domain_name_ = "default";
    }
    std::cout << "----------------------------------" << std::endl;

    if (!rclcpp::ok()) {
      rclcpp::init(argc, argv);

#if (ROS_DISTRO == foxy)
      for (auto &domain_config : domain_ids) {
        contexts_[domain_config.first] =
          create_context_with_domain_id(domain_config.second);
        nh_list_[domain_config.first] =
          create_node(node_name + "_" + std::to_string(domain_config.second),
                      domain_config.second,
                      contexts_[domain_config.first]);
      }
#else
      rclcpp::InitOptions context_options;
      for (auto &domain_config : domain_ids) {
        contexts_[domain_config.first] = std::make_shared<rclcpp::Context>();
        contexts_.auto_initialize_logging(true).set_domain_id(
          static_cast<std::size_t>(domain_config.second));
        contexts_[domain_config.first]->init(0, nullptr, context_options);
      }
      // Initialize one node in each domain
      rclcpp::NodeOptions node_options;
      for (auto &domain_config : domain_ids) {
        node_options.context(contexts_[domain_config.first]);
        nh_list_.push_back(std::make_shared<rclcpp::Node>(
          node_name + "_" + std::to_string(domain_config.second),
          node_options));
      }
#endif
    }
  }

public:
  /**
   * @brief getting a node handler
   * @see getHandler()
   */
  std::shared_ptr<rclcpp::Node> getHandler(
    const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    return it != nh_list_.end() ? nh_list_[find_domain] : nullptr;
  };

  std::unordered_map<std::string, std::shared_ptr<rclcpp::Node>> getHandlers() {
    return nh_list_;
  };

  std::string getDefaultDomainName(void){
    return default_domain_name_;
  }

  /**
   * @brief getting a node name
   * @see getName()
   */
  std::string getName(const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    return it != nh_list_.end() ? nh_list_[find_domain]->get_name() : "";
  }

  /**
   * @brief register a publisher
   * @see registerPublisher()
   * @param name a topic name
   * @param buffer buffer size
   */
  template <class M>
  void registerPublisher(const std::string &name,
                         const uint32_t &buffer,
                         const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    if (it != nh_list_.end()) {
      auto p = std::make_shared<PublishHandler<M>>(
        nh_list_[find_domain], name, buffer);
      pub_handlers_.insert({name, (std::any)p});
    }
  }

  /**
   * @brief register a subscriber
   * @see registerSubscriber()
   * @param name a topic name
   * @param buffer buffer size
   */
  template <class M>
  void registerSubscriber(const std::string &name,
                          const uint32_t &buffer,
                          const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    if (it != nh_list_.end()) {
      auto p = std::make_shared<SubscribeHandler<M>>(
        nh_list_[find_domain], name, buffer);
      sub_handlers_.insert({name, (std::any)p});
    }
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
                          void (*f)(const M &),
                          const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    if (it != nh_list_.end()) {
      auto p = std::make_shared<SubscribeHandler<M>>(
        nh_list_[find_domain], name, buffer);
      p->setCallbackFunc(f);
      sub_handlers_.insert({name, (std::any)p});
    }
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
                          T *obj,
                          const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    if (it != nh_list_.end()) {
      auto p = std::make_shared<SubscribeHandler<M>>(
        nh_list_[find_domain], name, buffer);
      p->setCallbackFunc(std::bind(f, obj, std::placeholders::_1));
      sub_handlers_.insert({name, (std::any)p});
    }
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
    }
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
    }
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
    }
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
    return std::nullopt;
  }

  /**
   * @brief getting a ros parameter
   * @see getParam()
   * @param name a parameter name
   * @param value the returned paramter
   */
  template <class M>
  bool getParam(const std::string &key,
                const M &value,
                const std::string &domain_name = "default") {
    auto find_domain =
      domain_name == "default" ? default_domain_name_ : domain_name;
    auto it = nh_list_.find(find_domain);
    if (it != nh_list_.end()) {
      rclcpp::Parameter parameter;
      bool flag = nh_list_[find_domain]->get_parameter(key, parameter);
      if (flag) value = parameter.get_value<M>();
      return flag;
    }
    return false;
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
    }
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
    }
  }

#if (ROS_DISTRO == foxy)
private:
  rclcpp::Context::SharedPtr create_context_with_domain_id(
    const std::size_t domain_id) {
    rcl_init_options_t rcl_init_options =
      rcl_get_zero_initialized_init_options();
    rcl_ret_t ret =
      rcl_init_options_init(&rcl_init_options, rcl_get_default_allocator());
    if (RCL_RET_OK != ret) {
      std::runtime_error("Failed to initialize rcl_init_options");
    }

    ret = rcl_init_options_set_domain_id(&rcl_init_options, domain_id);
    if (RCL_RET_OK != ret) {
      std::runtime_error("Failed to set domain ID to rcl_init_options");
    }

    rclcpp::InitOptions init_options(rcl_init_options);
    init_options.auto_initialize_logging(false);

    auto context = std::make_shared<rclcpp::Context>();
    context->init(0, nullptr, init_options);
    return context;
  }

  rclcpp::Node::SharedPtr create_node(const std::string &name,
                                      const std::size_t domain_id,
                                      rclcpp::Context::SharedPtr context) {
    if (context == nullptr) {
      context = create_context_with_domain_id(domain_id);
    }

    rclcpp::NodeOptions node_options;
    node_options.context(context)
      .use_global_arguments(false)
      .start_parameter_services(false)
      .start_parameter_event_publisher(false);

    return std::make_shared<rclcpp::Node>(name, node_options);
  }
#endif
private:
  // std::shared_ptr<rclcpp::Node> nh_;
  std::string default_domain_name_ = "default";
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Node>> nh_list_;
  std::unordered_map<std::string, std::shared_ptr<rclcpp::Context>> contexts_;
  std::map<std::string, std::any> pub_handlers_;
  std::map<std::string, std::any> sub_handlers_;
};

}  // namespace ROS2
}  // namespace add

#endif  // BASE_ROS2_NODE_HPP_ // NOLINT