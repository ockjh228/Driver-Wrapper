cmake_minimum_required(VERSION 3.8)
project(kinova_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Automatically find all .msg and .srv files
file(GLOB_RECURSE MESSAGE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
file(GLOB_RECURSE SERVICE_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*.srv)

# Ensure there are actually message and service files
if(MESSAGE_FILES OR SERVICE_FILES)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${MESSAGE_FILES}
    ${SERVICE_FILES}
    DEPENDENCIES builtin_interfaces std_msgs geometry_msgs
  )
endif()

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
