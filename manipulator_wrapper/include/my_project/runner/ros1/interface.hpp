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

namespace acelab {
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
