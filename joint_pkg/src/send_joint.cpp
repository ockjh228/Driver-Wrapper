#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

// namespace k_api = Kinova::Api;

using namespace std::chrono_literals;

class PubandSub
{
public:
  PubandSub(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
    kinematic_model_ = robot_model_loader_->getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    robot_state_->setToDefaultValues();

    publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

    subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/end_effector_pose", 10,
      std::bind(&PubandSub::pose_callback, this, std::placeholders::_1));

    joint_positions_subscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/manual_input", 10, std::bind(&PubandSub::joint_positions_callback, this, std::placeholders::_1));
  }

private:
  void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) 
  {
    Eigen::Isometry3d end_effector_pose = Eigen::Isometry3d::Identity();
    end_effector_pose.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    end_effector_pose.linear() = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();

    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("manipulator");
    std::vector<double> joint_values;
    double timeout = 1.0;

    auto start_t  = std::chrono::high_resolution_clock::now();

    bool found_ik = robot_state_->setFromIK(joint_model_group, end_effector_pose, timeout);

    auto end_t = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
    std::cout<<"Time taken: "<<duration<<" ms "<<std::endl;

    auto message = trajectory_msgs::msg::JointTrajectory();
    message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    trajectory_msgs::msg::JointTrajectoryPoint point;

    if (found_ik) {
      robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
      point.positions = joint_values;
      point.time_from_start = rclcpp::Duration::from_seconds(0.2);
      message.points.push_back(point);

      publisher_->publish(message);
      RCLCPP_INFO(node_->get_logger(), "Published joint trajectory based on IK solution from pose");
    }
    else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to find IK solution from pose");
    }
  }

  void joint_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("manipulator");
    std::vector<double> joint_values = msg->data;

    if (joint_values.size() != joint_model_group->getVariableCount()) {
      RCLCPP_ERROR(node_->get_logger(), "Received joint positions array size mismatch");
      return;
    }
    
    robot_state_->setJointGroupPositions(joint_model_group, joint_values);
    robot_state_->update();
    
    Eigen::Isometry3d end_effector_pose = robot_state_->getGlobalLinkTransform("end_effector_link");
    
    double timeout = 1.0;
    bool found_ik = robot_state_->setFromIK(joint_model_group, end_effector_pose, timeout);

    auto message = trajectory_msgs::msg::JointTrajectory();
    message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    trajectory_msgs::msg::JointTrajectoryPoint point;

    if (found_ik) {
      robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
      point.positions = joint_values;
      point.time_from_start = rclcpp::Duration::from_seconds(10);
      message.points.push_back(point);

      publisher_->publish(message);
      RCLCPP_INFO(node_->get_logger(), "Published joint trajectory based on IK solution from joint positions");
    } 
    else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to find IK solution from joint positions");
    }
  }


  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelPtr kinematic_model_;
  moveit::core::RobotStatePtr robot_state_;
  
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_positions_subscriber_;
  // rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_subscriber_;
  // rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr fault_reset_subscriber_;
  // std::shared_ptr<kortex_driver::KortexMultiInterfaceHardware> hardware_interface_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = rclcpp::Node::make_shared("joint_info", node_options);
  auto pub_and_sub = std::make_shared<PubandSub>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}


/***********************************************************/
//Below Code is for Keyboard Control

// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/robot_state.h>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>
// #include "rclcpp/rclcpp.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
// #include "trajectory_msgs/msg/joint_trajectory_point.hpp"
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// using namespace std::chrono_literals;

// class PubandSub
// {
// public:
//   PubandSub(const rclcpp::Node::SharedPtr& node)
//   {
//     node_ = node;

//     robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_);
//     kinematic_model_ = robot_model_loader_->getModel();
//     robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
//     robot_state_->setToDefaultValues();

//     publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

//     subscriber_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>("/end_effector_pose", 10,
//       std::bind(&PubandSub::pose_callback, this, std::placeholders::_1));

//     joint_positions_subscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
//       "/manual_input", 10, std::bind(&PubandSub::joint_positions_callback, this, std::placeholders::_1));
//   }

// private:
//   void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) //for /end_effector_pose subscription
//   {
//     Eigen::Isometry3d end_effector_pose = Eigen::Isometry3d::Identity();
//     end_effector_pose.translation() = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
//     end_effector_pose.linear() = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z).toRotationMatrix();

//     const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("manipulator");
//     std::vector<double> joint_values;
//     double timeout = 1.0;
//     bool found_ik = robot_state_->setFromIK(joint_model_group, end_effector_pose, timeout);

//     auto message = trajectory_msgs::msg::JointTrajectory();
//     message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

//     trajectory_msgs::msg::JointTrajectoryPoint point;

//     if (found_ik) {
//       robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
//       point.positions = joint_values;
//       point.time_from_start = rclcpp::Duration::from_seconds(3);
//       message.points.push_back(point);

//       publisher_->publish(message);
//       RCLCPP_INFO(node_->get_logger(), "Published joint trajectory based on IK solution from pose");
//     } else {
//       RCLCPP_ERROR(node_->get_logger(), "Failed to find IK solution from pose");
//       // Print the joint positions of the last point
//       if (!message.points.empty()) {
//         const auto& last_point = message.points.back();
//         std::cout << "Joint positions: ";
//         for (const auto& pos : last_point.positions) {
//           std::cout << pos << " ";
//         }
//         std::cout << std::endl;
//       }
//     }
//   }

//   void joint_positions_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
//   {
//     const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("manipulator");
//     std::vector<double> joint_values = msg->data;

//     if (joint_values.size() != joint_model_group->getVariableCount()) {
//       RCLCPP_ERROR(node_->get_logger(), "Received joint positions array size mismatch");
//       return;
//     }
    
//     robot_state_->setJointGroupPositions(joint_model_group, joint_values);
//     robot_state_->update();
    
//     Eigen::Isometry3d end_effector_pose = robot_state_->getGlobalLinkTransform("end_effector_link");
    
//     double timeout = 1.0;
//     bool found_ik = robot_state_->setFromIK(joint_model_group, end_effector_pose, timeout);

//     auto message = trajectory_msgs::msg::JointTrajectory();
//     message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

//     trajectory_msgs::msg::JointTrajectoryPoint point;

//     if (found_ik) {
//       robot_state_->copyJointGroupPositions(joint_model_group, joint_values);
//       point.positions = joint_values;
//       point.time_from_start = rclcpp::Duration::from_seconds(10);
//       message.points.push_back(point);

//       publisher_->publish(message);
//       RCLCPP_INFO(node_->get_logger(), "Published joint trajectory based on IK solution from joint positions");
//     } else {
//       RCLCPP_ERROR(node_->get_logger(), "Failed to find IK solution from joint positions");
//     }
//   }

//   std::shared_ptr<rclcpp::Node> node_;
//   std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
//   moveit::core::RobotModelPtr kinematic_model_;
//   moveit::core::RobotStatePtr robot_state_;
//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
//   rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
//   rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_positions_subscriber_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("pub_and_sub_node");
//   auto pub_and_sub = std::make_shared<PubandSub>(node);
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
// }
