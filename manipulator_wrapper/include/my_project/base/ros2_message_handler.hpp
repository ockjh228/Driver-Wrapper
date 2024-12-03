/**
 * @file ros1_message_handler.hpp
 * @author kyungpyo kim (kyungpyo.kim@acelab.ai)
 * @author wonteak lim (wonteak.lim@acelab.ai)
 * @brief
 * @version 1.0 (2021-11-14) modified by kyungpyo kim
 * @date 2022-11-14
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef BASE_ROS2_MESSAGE_HANDLER_HPP_  // NOLINT
#define BASE_ROS2_MESSAGE_HANDLER_HPP_  // NOLINT

#include <rclcpp/rclcpp.hpp>

#include <functional>
#include <mutex>  // NOLINT
#include <string>

namespace acelab {
namespace ROS2 {

/**
 * @class Msg
 * @brief ros message management
 */
template <class M>
class Msg {
public:
  /**
   * @brief a constructor
   * @see Msg()
   * @param name a message name
   * @param buffer buffer
   */
  Msg(const std::string &name, const uint32_t &buffer)
      : name_(name), buffer_(buffer) {}

public:
  /**
   * @brief getting a message name
   * @see getName()
   * @return a message name
   */
  std::string getName() const { return name_; }

  /**
   * @brief getting a message
   * @see getMessage()
   * @return a message
   */
  M getMessage() const {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    return message_;
  }

  /**
   * @brief setting a message
   * @see setMessage()
   * @param m a message.
   */
  void setMessage(const M &m) {
    std::lock_guard<std::mutex> lock_guard(mutex_);
    message_ = m;
  }

private:
  mutable std::mutex mutex_;
  std::string name_;
  unsigned int buffer_;
  M message_;
};
/**
 * @class SubscribeHandler
 * @brief handle a ros subscriber
 */
template <class M>
class SubscribeHandler {
public:
  /**
   * @brief a constructor
   * @see SubscribeHandler()
   * @param name a message name
   * @param buffer buffer
   */
  SubscribeHandler(std::shared_ptr<rclcpp::Node> nh, const std::string &name, const uint32_t &buffer)
      : nh_(nh), message_(name, buffer) {

    callback_group_ = nh_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    auto opt = rclcpp::SubscriptionOptions();
    opt.callback_group = callback_group_;

    subscriber_ = nh_->create_subscription<M>(
      name, buffer, std::bind(&SubscribeHandler<M>::callbackFunc, this, std::placeholders::_1), opt);
  }

  /**
   * @brief getting a message
   * @see getMessage()
   * @return a message
   */
  M getMessage() const { return message_.getMessage(); }

  /**
   * @brief setting a subscribe callback function
   * @see setCallbackFunc()
   * @param f callback function
   */
  void setCallbackFunc(void (*f)(const M &)) {
    custom_callback_ = f;
    custom_callback_(M());
  }

  /**
   * @brief setting a subscribe callback function
   * @see setCallbackFunc()
   * @param f callback function
   */
  void setCallbackFunc(const std::function<void(const M &)> f) {
    custom_callback_ = f;
  }

  /**
   * @brief unregister a subscriber
   * @see unregister()
   */
  void unregister() { subscriber_.reset(); }

private:
  /**
   * @brief defaulf callbackfunctin for copying a message
   * @see callbackFunc()
   * @param msg a message from a subscribe callback
   */
#if(ROS_DISTRO == foxy)
  void callbackFunc(const std::shared_ptr<M> msg) {
    message_.setMessage(*msg.get());
    if (custom_callback_ != nullptr) {
      this->custom_callback_(*msg.get());
    }
  }
#else
  void callbackFunc(const M& msg) {
    message_.setMessage(msg);
    if (custom_callback_ != nullptr) {
      this->custom_callback_(msg);
    }
  }
#endif

private:
  std::shared_ptr<rclcpp::Node> nh_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  Msg<M> message_;
  typename rclcpp::Subscription<M>::SharedPtr subscriber_;
  std::function<void(const M &)> custom_callback_;
};

/**
 * @class PublishHandler
 * @brief handle a ros publisher
 */
template <class M>
class PublishHandler {
public:
  /**
   * @brief a constructor
   * @see PublishHandler()
   * @param name a message name
   * @param buffer buffer
   */
  PublishHandler(std::shared_ptr<rclcpp::Node> nh, const std::string &name, const uint32_t &buffer)
      : nh_(nh), message_(name, buffer) {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
                .reliable()            
                .durability_volatile(); 
    publisher_ = nh_->create_publisher<M>(name, qos);  }

  /**
   * @brief setting a message
   * @see setMessage()
   * @param m a message.
   */
  void setMessage(const M &m) { message_.setMessage(m); }

  /**
   * @brief publish a message
   * @see publish()
   */
  void publish() { publisher_->publish(message_.getMessage()); }

  /**
   * @brief unregister a publisher
   * @see unregister()
   */
  void unregister() { publisher_.reset(); }

private:
  std::shared_ptr<rclcpp::Node> nh_;
  typename rclcpp::Publisher<M>::SharedPtr publisher_;
  Msg<M> message_;
};
}  // namespace ROS1
}  // namespace acelab

#endif  // BASE_ROS2_MESSAGE_HANDLER_HPP_  // NOLINT
