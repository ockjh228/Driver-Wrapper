/**
 * @file main.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-08
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#include <iostream>


#include "my_project/core/my_app_runnable.hpp"
#ifdef ROS2_BUILD
#include "my_project/runner/ros2/ros2_runner.hpp"
#else
#include "my_project/runner/ros1/ros_runner.hpp"
#endif

int main(int argc, char **argv) {
  /**
   * @brief get config file and logging path from command line
   *
   */
  if (argc < 3) {
    std::cout << "Usage: " << argv[0] << " <config_file> <loging_path>"
              << std::endl;

    for (int i = 0; i < argc; i++) {
      std::cout << "argv[" << i << "] = " << argv[i] << std::endl;
    }
    return 1;
  }
  std::string config_file = argv[1];
  std::string logging_dir = argv[2];

  /**
   * instantiate runnable and runner
   * by calling process(), the runner will start the task loop
   */
  auto my_app_runnable = std::make_shared<acelab::MyAppRunnable>(
    "MyAppRunnable", config_file, logging_dir);
  auto ros_runner =
    std::make_shared<acelab::ROS::Runner>(my_app_runnable, argc, argv);
  ros_runner->process();

  return 0;
}
