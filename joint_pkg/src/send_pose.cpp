#include <filesystem>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <string>
#include <map>
// #include <keyboard_msgs/msg/key.hpp>

using namespace std::chrono_literals;

class MinimalPublisher
{                      
public:
  MinimalPublisher(const rclcpp::Node::SharedPtr& node)
  {
    node_ = node;

    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/end_effector_pose", 10);

    twist_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "twist_cmd", 10, std::bind(&MinimalPublisher::twist_callback, this, std::placeholders::_1));
  }

  void set_pose(const geometry_msgs::msg::PoseStamped& pose){
    current_pose_ = pose;
  }

 void update_and_publish_pose(){
    if (!twist_received_ || (twist_msg_.linear.x == 0.0 && twist_msg_.linear.y == 0.0 && 
        twist_msg_.linear.z == 0.0 && twist_msg_.angular.x == 0.0 && 
        twist_msg_.angular.y == 0.0 && twist_msg_.angular.z == 0.0)) {
        return;  // Skip publishing if no valid input
    }  // Only update if a twist message was received
    
    // Start with the current pose
    auto updated_pose = current_pose_;
    
    // Update position based on linear twist (direct translation)
    updated_pose.pose.position.x += twist_msg_.linear.x;
    updated_pose.pose.position.y += twist_msg_.linear.y;
    updated_pose.pose.position.z += twist_msg_.linear.z;

    // Convert angular twist to rotation and apply it
    Eigen::AngleAxisf rollAngle(-twist_msg_.angular.x, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf pitchAngle(twist_msg_.angular.y, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf yawAngle(twist_msg_.angular.z, Eigen::Vector3f::UnitZ());
    Eigen::Quaternionf angularQuaternion = yawAngle * pitchAngle * rollAngle;


    // Combine current orientation with angular twist
    Eigen::Quaternionf currentQuaternion(
        current_pose_.pose.orientation.w,
        current_pose_.pose.orientation.x,
        current_pose_.pose.orientation.y,
        current_pose_.pose.orientation.z);

    // std::cout<<"---before"<<std::endl;
    // std::cout<<current_pose_.pose.orientation.w<<std::endl;
    // std::cout<<current_pose_.pose.orientation.x<<std::endl;
    // std::cout<<current_pose_.pose.orientation.y<<std::endl;
    // std::cout<<current_pose_.pose.orientation.z<<std::endl;    
    
    Eigen::Quaternionf updatedQuaternion = currentQuaternion * angularQuaternion;

    updatedQuaternion.normalize();

    // Update orientation
    updated_pose.pose.orientation.x = updatedQuaternion.x();
    updated_pose.pose.orientation.y = updatedQuaternion.y();
    updated_pose.pose.orientation.z = updatedQuaternion.z();
    updated_pose.pose.orientation.w = updatedQuaternion.w();

    // std::cout<<"---after---"<<std::endl;
    // std::cout<<updated_pose.pose.orientation.w<<std::endl;
    // std::cout<<updated_pose.pose.orientation.x<<std::endl;
    // std::cout<<updated_pose.pose.orientation.y<<std::endl;
    // std::cout<<updated_pose.pose.orientation.z<<std::endl;      

    publisher_->publish(updated_pose);

    twist_received_ = false;  // Reset the flag after processing
}

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
    twist_msg_ = *msg;
    twist_received_ = true;  // Set the flag indicating a new twist message was received
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;

  geometry_msgs::msg::PoseStamped current_pose_;
  geometry_msgs::msg::Twist twist_msg_;
  bool twist_received_ = false;  // Flag to check if a new twist message was received
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node =
    rclcpp::Node::make_shared("pose_info", node_options);

  auto minimal_publisher = std::make_shared<MinimalPublisher>(node);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "manipulator";
  moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

  rclcpp::Rate loop_rate(10); // loop for 10hz

  while (rclcpp::ok()) {
    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
    minimal_publisher->set_pose(current_pose);
    // Update pose if a twist message was received
    minimal_publisher->update_and_publish_pose();
    loop_rate.sleep();
  }

  return 0;
}



/***********************************************************/
//Below Code is for Keyboard control

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <keyboard_msgs/msg/key.hpp>d
// #include <Eigen/Dense>
// #include <map>
// #include <set>
// #include <vector>
// #include <moveit/move_group_interface/move_group_interface.h>

// class MinimalPublisher
// {
// public:
//   MinimalPublisher(const rclcpp::Node::SharedPtr& node)
//   : joint_data_updated_(false), key_pressed_(false)
//   {
//     node_ = node;

//     publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("/end_effector_pose", 10);

//     keydown_subscriber_ = node_->create_subscription<keyboard_msgs::msg::Key>(
//       "keydown", 10, std::bind(&MinimalPublisher::keydown_callback, this, std::placeholders::_1));

//     keyup_subscriber_ = node_->create_subscription<keyboard_msgs::msg::Key>(
//       "keyup", 10, std::bind(&MinimalPublisher::keyup_callback, this, std::placeholders::_1));

//     for(int key : valid_keys){
//       key_flags_[key] = false;
//     }
//   }

//   void set_pose(const geometry_msgs::msg::PoseStamped& pose){
//     current_pose_ = pose;
//   }

//   void publish_message()
//   {
//     if (!key_pressed_) {
//       return; // Only publish if a key is pressed
//     }

//     auto message = geometry_msgs::msg::PoseStamped();
//     message.header = current_pose_.header;

//     float x = current_pose_.pose.position.x;
//     float y = current_pose_.pose.position.y;
//     float z = current_pose_.pose.position.z;

//     auto euler = convert_quaternion_to_euler(current_pose_.pose.orientation);
//     float roll = euler[0];
//     float pitch = euler[1];
//     float yaw = euler[2];
//     float move_step = 0.05f;

//     // Update the pose based on key flags
//     if (key_flags_[119]) z += move_step; // w
//     if (key_flags_[115]) z -= move_step; // s
//     if (key_flags_[97])  x -= move_step; // a
//     if (key_flags_[100]) x += move_step; // d
//     if (key_flags_[113]) y += move_step; // q
//     if (key_flags_[101]) y -= move_step; // e
//     if (key_flags_[257]) roll += move_step; // 1
//     if (key_flags_[260]) roll -= move_step; // 4
//     if (key_flags_[258]) pitch += move_step; // 2
//     if (key_flags_[261]) pitch -= move_step; // 5
//     if (key_flags_[259]) yaw += move_step; // 3
//     if (key_flags_[262]) yaw -= move_step; // 6

//     Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
//     Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
//     Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());
//     Eigen::Quaternionf quaternion = yawAngle * pitchAngle * rollAngle;

//     message.pose.position.x = x;
//     message.pose.position.y = y;
//     message.pose.position.z = z;
//     message.pose.orientation.x = quaternion.x();
//     message.pose.orientation.y = quaternion.y();
//     message.pose.orientation.z = quaternion.z();
//     message.pose.orientation.w = quaternion.w();

//     publisher_->publish(message);
//     joint_data_updated_ = false; // Reset the flag after publishing
//     std::cout << "Published pose message" << std::endl;
//   }

//   bool is_valid_key(int key_code) {
//     return valid_keys.find(key_code) != valid_keys.end();
//   }

//   std::vector<float> convert_quaternion_to_euler(const geometry_msgs::msg::Quaternion& quaternion) {
//     std::vector<float> euler(3);

//     // roll (x-axis rotation)
//     float sinr_cosp = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
//     float cosr_cosp = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
//     euler[0] = std::atan2(sinr_cosp, cosr_cosp);

//     // pitch (y-axis rotation)
//     float sinp = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
//     euler[1] = std::abs(sinp) >= 1 ? std::copysign(M_PI / 2, sinp) : std::asin(sinp);

//     // yaw (z-axis rotation)
//     float siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
//     float cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
//     euler[2] = std::atan2(siny_cosp, cosy_cosp);

//     return euler;
//   }

// private:
//   void keydown_callback(const keyboard_msgs::msg::Key::SharedPtr msg){
//     if (is_valid_key(msg->code)){
//       key_flags_[msg->code] = true;
//       key_pressed_ = true;
//       publish_message();  // Call publish_message on keydown
//     }
//   }

//   void keyup_callback(const keyboard_msgs::msg::Key::SharedPtr msg){
//     if (is_valid_key(msg->code)){
//       key_flags_[msg->code] = false;
//       key_pressed_ = std::any_of(key_flags_.begin(), key_flags_.end(), [](const auto& pair){ return pair.second; });
//       if (!key_pressed_) {
//         publish_message();  // Call publish_message on keyup if no keys are pressed
//       }
//     }
//   }

//   std::shared_ptr<rclcpp::Node> node_;
//   rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
//   rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keydown_subscriber_;
//   rclcpp::Subscription<keyboard_msgs::msg::Key>::SharedPtr keyup_subscriber_;

//   geometry_msgs::msg::PoseStamped current_pose_;
//   std::map<int, bool> key_flags_;
//   std::set<int> valid_keys = {119, 115, 97, 100, 113, 101, 257, 258, 259, 260, 261, 262};
//   bool joint_data_updated_;
//   bool key_pressed_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);  
//   rclcpp::NodeOptions node_options;
//   node_options.automatically_declare_parameters_from_overrides(true);
//   auto node = rclcpp::Node::make_shared("pose_info", node_options);

//   auto minimal_publisher = std::make_shared<MinimalPublisher>(node);

//   rclcpp::executors::SingleThreadedExecutor executor;
//   executor.add_node(node);
//   std::thread([&executor]() { executor.spin(); }).detach();

//   static const std::string PLANNING_GROUP = "manipulator";
//   moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);

//   rclcpp::Rate loop_rate(10); // loop for 10hz

//   while (rclcpp::ok()) {
//     geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
//     minimal_publisher->set_pose(current_pose);
//     loop_rate.sleep();
//   }

//   return 0;
// }
