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

#ifndef BASE_ROS1_MESSAGE_HANDLER_HPP_  // NOLINT
#define BASE_ROS1_MESSAGE_HANDLER_HPP_  // NOLINT

#include <ros/ros.h>

#include <functional>
#include <mutex>  // NOLINT
#include <string>

namespace acelab {
namespace ROS1 {

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
  SubscribeHandler(const std::string &name, const uint32_t &buffer)
      : message_(name, buffer) {
    subscriber_ = nh_.subscribe(name,
                                buffer,
                                &SubscribeHandler<M>::callbackFunc,
                                this,
                                ros::TransportHints().tcpNoDelay(true));
  }

  /**
   * @brief getting a message
   * @see getMessage()
   * @return a message
   */
  M getMessage() const { return message_.getMessage(); }

  /**
   * @brief resetting a message
   * @see resetMessage()
   */
  void resetMessage() { message_.setMessage(M()); }

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
  void unregister() { subscriber_.shutdown(); }

private:
  /**
   * @brief defaulf callbackfunctin for copying a message
   * @see callbackFunc()
   * @param msg a message from a subscribe callback
   */
  void callbackFunc(const M &msg) {
    message_.setMessage(msg);
    if (custom_callback_ != nullptr) {
      this->custom_callback_(msg);
    }
  }

private:
  Msg<M> message_;
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
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
  PublishHandler(const std::string &name, const uint32_t &buffer)
      : message_(name, buffer) {
    publisher_ = nh_.advertise<M>(name, buffer);
  }

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
  void publish() { publisher_.publish(message_.getMessage()); }

  /**
   * @brief unregister a publisher
   * @see unregister()
   */
  void unregister() { publisher_.shutdown(); }

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  Msg<M> message_;
};
}  // namespace ROS1
}  // namespace acelab

#endif  // BASE_ROS1_MESSAGE_HANDLER_HPP_  // NOLINT
