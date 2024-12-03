# cmake_minimum_required(VERSION 3.8)
# project(joint_pkg)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(moveit_core REQUIRED)
# find_package(moveit_ros_planning REQUIRED)
# find_package(moveit_ros_planning_interface REQUIRED)
# find_package(trajectory_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2_eigen REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(keyboard_msgs REQUIRED)
# find_package(SDL2 REQUIRED)

# # Add joint_pub node
# add_executable(joint_pub src/send_joint.cpp)
# ament_target_dependencies(joint_pub rclcpp std_msgs moveit_ros_planning_interface trajectory_msgs geometry_msgs tf2_eigen Eigen3)

# # Add pose_pub node
# add_executable(pose_pub src/send_pose.cpp)
# ament_target_dependencies(pose_pub rclcpp std_msgs moveit_ros_planning_interface trajectory_msgs keyboard_msgs geometry_msgs tf2_eigen Eigen3)

# # Add user_input node
# add_executable(user_input src/send_user_input.cpp)
# ament_target_dependencies(user_input rclcpp std_msgs keyboard_msgs SDL2)

# install(TARGETS 
#   joint_pub
#   pose_pub
#   user_input
#   DESTINATION lib/${PROJECT_NAME})
  
# install(DIRECTORY 
#   launch
#  DESTINATION share/${PROJECT_NAME})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

# ament_package()

###########################3

cmake_minimum_required(VERSION 3.8)
project(joint_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(keyboard_msgs REQUIRED)
find_package(SDL2 REQUIRED)

# Set the Kortex SDK directory
set(KORTEX_SDK_DIR "/home/alex/workspace/demo_ws/src/kortex_sdk")


# Include directories
include_directories(
  ${KORTEX_SDK_DIR}/include
  ${KORTEX_SDK_DIR}/include/client_stubs
  ${KORTEX_SDK_DIR}/include/messages
  ${KORTEX_SDK_DIR}/include/common
  ${KORTEX_SDK_DIR}/include/client
)

# Link directories
link_directories(${KORTEX_SDK_DIR}/lib/release)

# Add joint_pub node
add_executable(joint_pub src/send_joint.cpp)
ament_target_dependencies(joint_pub rclcpp std_msgs moveit_ros_planning_interface trajectory_msgs geometry_msgs tf2_eigen Eigen3)

# Add pose_pub node
add_executable(pose_pub src/send_pose.cpp)
ament_target_dependencies(pose_pub rclcpp std_msgs moveit_ros_planning_interface trajectory_msgs  geometry_msgs tf2_eigen Eigen3)

# Add user_input node

# Link the Kortex SDK library to the joint_pub node
target_link_libraries(joint_pub 
                      # KortexApiCpp
                      )

install(TARGETS 
  joint_pub
  pose_pub
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
