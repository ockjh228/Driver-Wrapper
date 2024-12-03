/**
 * @file interface.hpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef INCLUDE_MY_PROJECT_RUNNER_INTERFACE_HPP_
#define INCLUDE_MY_PROJECT_RUNNER_INTERFACE_HPP_

#include <memory>

#include "kinova_msgs/msg/state.hpp"
#include "kinova_msgs/msg/status.hpp"
#include "kinova_msgs/msg/action_list.hpp"
#include "kinova_msgs/msg/protection_zone_list.hpp"
#include "std_msgs/msg/string.hpp"

namespace acelab {

struct ManipulatorStatus {
  kinova_msgs::msg::State state;
  kinova_msgs::msg::Status status;
  std::vector<std::string> action_list;
  std::vector<std::string> protection_zone_list;
};

class Interface {
public:
  Interface() = default;
  virtual ~Interface() = default;

public:
  template <class From, class To>
  static To convert(const From& from);

};
}  // namespace acelab
#endif  // INCLUDE_MY_PROJECT_RUNNER_INTERFACE_HPP_
