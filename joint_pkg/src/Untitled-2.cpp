// #include <moveit/robot_model/robot_model.h>
// #include <moveit/robot_model_loader/robot_model_loader.h>
// #include <moveit/robot_state/robot_state.h>

// #include <chrono>
// #include <functional>
// #include <memory>
// #include <string>

// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp"
// #include "trajectory_msgs/msg/joint_trajectory_point.hpp"


// using namespace std::chrono_literals;

// /* This example creates a subclass of Node and uses std::bind() to register a
//  * member function as a callback from the timer. */
// // Every "This"  in the code is referring to the node.

// class MinimalPublisher : public rclcpp::Node
// {
// public:
//   MinimalPublisher() 
//   : Node("minimal_publisher")
//   {
//     robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(std::static_pointer_cast<rclcpp::Node>(this->shared_from_this()), "robot_description");
//     kinematic_model_ = robot_model_loader_->getModel();

//     robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
//     robot_state_->setToDefaultValues();

//     publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);

//     // Directly call the function to publish the message
//     publish_message();

//     /**********************************/
//     // robot_model_loader::RobotModelLoader robot_model_loader(Node);

//     // robot_model_loader::RobotModelLoader robot_model_loader(std::make_shared<rclcpp::Node>(*this));

//     // const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();

    
//     // // kinematic_model_ = robot_model_loader_->getModel();
//     // robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
//     // moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(kinematic_model));
//     // robot_state_->setToDefaultValues();

//     // publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
//     // publish_message();
//   }

// private:
//   // void timer_callback()
//   // {
//   //   robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(this, "robot_description");
//   //   kinematic_model_ = robot_model_loader_->getModel();
//   //   robot_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
//   //   robot_state_->setToDefaultValues();

//   //   publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/joint_trajectory_controller/joint_trajectory", 10);
//   //   publish_message();
//   // }

//   void publish_message()
//   {
//   //     int count = 0;
//   // while(1){
//   //   count++;
//   //   std::cout << "count: " << count << std::endl;
//   //   if(count==384243)
//   //     break;
//   // }
//     const moveit::core::JointModelGroup* joint_model_group = kinematic_model_->getJointModelGroup("manipulator");
//     std::vector<double> joint_values;

//     robot_state_->setToRandomPositions(joint_model_group);
//     const Eigen::Isometry3d& end_effector_state = robot_state_->getGlobalLinkTransform("end_effector_link");  

//     //robot_state_->copyJointGroupPositions(joint_model_group, joint_values);

//     auto message = trajectory_msgs::msg::JointTrajectory();
//     message.joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

//     double timeout = 0.1;
//     bool found_ik = robot_state_->setFromIK(joint_model_group, end_effector_state, timeout);
      
//     trajectory_msgs::msg::JointTrajectoryPoint point;
//     if(found_ik){
//       robot_state_->setFromIK(joint_model_group,end_effector_state, timeout);
//       point.positions = joint_values;
//     }
//     else {
//       RCLCPP_ERROR(this->get_logger(), "Did not find IK solution");
//       point.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};  // Default values if IK fails
//     }

//     // trajectory_msgs::msg::JointTrajectoryPoint point;
//     // point.positions = joint_values;  // 계산된 조인트 각도를 할당

//     point.time_from_start = rclcpp::Duration(10s);
//     message.points.push_back(point);

//     RCLCPP_INFO(this->get_logger(), "Publishing: joint_trajectory");
//     publisher_->publish(message);
//   }

//   // rclcpp::TimerBase::SharedPtr timer_;
//   rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
//   std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
//   moveit::core::RobotModelPtr kinematic_model_;
//   moveit::core::RobotStatePtr robot_state_;
//   // rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
//   // size_t count_;
// };

// int main(int argc, char * argv[])
// {
//   rclcpp::init(argc, argv);
//   rclcpp::spin_some(std::make_shared<MinimalPublisher>());
//   // rclcpp::spin(std::make_shared<MinimalPublisher>());
//   rclcpp::shutdown();
//   return 0;
// }
