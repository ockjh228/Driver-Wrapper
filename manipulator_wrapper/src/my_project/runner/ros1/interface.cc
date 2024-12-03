/**
 * @file interface.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#include "my_project/runner/ros1/interface.hpp"

#include <std_msgs/Bool.h>
#include <std_msgs/Int64.h>

namespace acelab {

template <>
std_msgs::Bool Interface::convert(const bool &data) {
  std_msgs::Bool msg;
  msg.data = data;
  return msg;
}

template <>
bool Interface::convert(const std_msgs::Bool &msg) {
  return msg.data;
}

template <>
std_msgs::Int64 Interface::convert(const int64_t &data) {
  std_msgs::Int64 msg;
  msg.data = data;
  return msg;
}

template <>
int64_t Interface::convert(const std_msgs::Int64 &msg) {
  return msg.data;
}

}  // namespace acelab
