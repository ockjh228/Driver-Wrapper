cmake_minimum_required(VERSION 3.5)
project(manipulator_wrapper)

# ##########
# # Setup ##
# ##########
set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 17)

if($ENV{ROS_VERSION} MATCHES 1)

find_package(catkin REQUIRED COMPONENTS

  # ros dependency
  roscpp

  # ros message dependency
  std_msgs

  # user defined message dependency

  # brain interface dependency
  health_checker
)

find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(add_sugv_msgs REQUIRED) 
find_package(kinova_msgs REQUIRED)
find_package(control_msgs REQUIRED)

catkin_package(
  INCLUDE_DIRS include
)

# ##########
# # Build ##
# ##########
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${SPDLOG_INCLUDE_DIRS}
  include
  include/my_project
)

set(RUNNER
  src/my_project/runner/ros1/ros_runner.cc
  src/my_project/runner/ros1/interface.cc
)
set(MY_APP
  src/my_project/core/my_app_runnable.cc
)
set(MAIN_SRC_FILE
  src/my_project/main.cc
)

add_executable(${PROJECT_NAME}
  ${RUNNER}
  ${MY_APP}
  ${MAIN_SRC_FILE}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  yaml-cpp
  stdc++fs
  Threads::Threads
)

# ############
# #  Test   ##
# ############

if (CATKIN_ENABLE_TESTING)

  catkin_add_gtest(${PROJECT_NAME}_test test/test_template_core.cc
    ${MY_APP})

  target_link_libraries(${PROJECT_NAME}_test 
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    yaml-cpp
    stdc++fs
    Threads::Threads)
endif()

# ############
# # Install ##
# ############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES LICENSE CHANGELOG.md
  DESTINATION ${CATKIN_PACKAGE_ETC_DESTINATION}
)

elseif($ENV{ROS_VERSION} MATCHES 2)

add_definitions( -DROS2_BUILD )
add_definitions( -DROS_DISTRO=$ENV{ROS_DISTRO} )

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(add_sugv_msgs REQUIRED) 
find_package(kinova_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)
find_package(spdlog REQUIRED)
find_package(Boost REQUIRED COMPONENTS system chrono thread)
find_package(tf2 REQUIRED)
# find_package(tf2_geometry_msgs REQUIRED)

# ##########
# # Build ##
# ##########
include_directories(
  ${SPDLOG_INCLUDE_DIRS}
  include
  include/my_project
  ${tf2_INCLUDE_DIRS}
  # ${tf2_geometry_msgs_INCLUDE_DIRS}
)

set(RUNNER
  src/my_project/runner/ros2/ros2_runner.cc
  src/my_project/runner/ros2/interface.cc
)
set(MY_APP
  src/my_project/core/my_app_runnable.cc
)
set(MAIN_SRC_FILE
  src/my_project/main.cc
)

add_executable(${PROJECT_NAME}
  ${RUNNER}
  ${MY_APP}
  ${MAIN_SRC_FILE}
)
target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  yaml-cpp
  stdc++fs
  Threads::Threads
  spdlog::spdlog
  tf2::tf2
  # tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  geometry_msgs
  add_sugv_msgs
  kinova_msgs
  control_msgs
  yaml-cpp
  std_msgs
  Boost
  # tf2
  # tf2_geometry_msgs
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}_hooks.dsv.in")

ament_package()

# ############
# # Install ##
# ############
install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE CHANGELOG.md
  DESTINATION share/${PROJECT_NAME}
)

endif()
