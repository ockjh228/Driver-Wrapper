cmake_minimum_required(VERSION 3.8)
project(add_sugv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if($ENV{ROS_VERSION} MATCHES 2)
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  # find message dependencies
  find_package(std_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)

  # generate messages
  find_package(rosidl_default_generators REQUIRED)

  file(GLOB_RECURSE MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
  file(GLOB_RECURSE SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MESSAGE_FILES} ${SERVICE_FILES}
    DEPENDENCIES builtin_interfaces std_msgs sensor_msgs nav_msgs geometry_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)

  ament_package()

else()
  message(WARNING "This package is only for ROS 2")
endif()
