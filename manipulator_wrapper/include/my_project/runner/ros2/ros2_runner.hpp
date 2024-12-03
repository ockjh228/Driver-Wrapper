/**
 * @file ros_runnable.hpp
 * * @author Alex Ock (ockjh228@berekeley.edu)
 * @brief
 * @version 0.1
 * @date 2024-10-11
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#ifndef INCLUDE_MY_PROJECT_RUNNER_ROS_RUNNER_HPP_
#define INCLUDE_MY_PROJECT_RUNNER_ROS_RUNNER_HPP_

#include <iostream>
#include <memory>
#include <string>

#include "my_project/base/ros2_runner.hpp"
#include "my_project/core/my_app_runnable.hpp"
#include "my_project/runner/ros2/interface.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/string.hpp"

#include "add_sugv_msgs/msg/twist_cmd.hpp"
#include "add_sugv_msgs/msg/joint_cmd.hpp"
#include "add_sugv_msgs/msg/preset_cmd.hpp"
#include "add_sugv_msgs/msg/protection_zone_cmd.hpp"
#include "add_sugv_msgs/msg/emergency_stop_cmd.hpp"
#include "add_sugv_msgs/msg/clear_fault_cmd.hpp"
#include "add_sugv_msgs/msg/gripper_cmd.hpp"
#include "add_sugv_msgs/msg/manipulator_status.hpp"
#include "add_sugv_msgs/msg/preset_info.hpp"
#include "add_sugv_msgs/msg/preset_info_array.hpp"
#include "add_sugv_msgs/msg/protection_zone_info.hpp"
#include "add_sugv_msgs/msg/protection_zone_info_array.hpp"
#include "add_sugv_msgs/srv/register_service.hpp"

#include "kinova_msgs/msg/state.hpp"
#include "kinova_msgs/msg/status.hpp"
#include "kinova_msgs/msg/create_action.hpp"
#include "kinova_msgs/msg/action_info.hpp"
#include "kinova_msgs/msg/action_info_array.hpp"
#include "kinova_msgs/msg/protection_zone.hpp"
#include "kinova_msgs/msg/protection_zone_info.hpp"
#include "kinova_msgs/msg/protection_zone_info_array.hpp"
#include "kinova_msgs/msg/action_list.hpp"
#include "kinova_msgs/msg/protection_zone_list.hpp"

namespace acelab {
  namespace ROS {

    class Runner : public ROS2::Runner, public Interface {
    public:
      Runner(std::shared_ptr<MyAppRunnable> runnable,
            int argc,
            char **argv,
            std::unordered_map<std::string, int> domain_ids = {{"integrated", 0}, {"mission", 1}});
      ~Runner();

      /**
       * override virtual functions of ROS1::Runner
       * these functions are called by the ROS1::Runner::loop()
       *
       * init() is called before starting task loop
       *
       * in the task loop, start(), end() and diagnose() are called in order
       *
       * start(): set value to runnable_ using ROS interface
       * end(): publish using ROS interface
       * diagnose(): publish using ROS diagnose interface
       */
      void init() override;
      void start() override;
      void end() override;
      void diagnose() override;

    private:
      /*
        Custom Callback Function
      */
      // Control Commands
      void callbackEmergencyStop(const add_sugv_msgs::msg::EmergencyStopCmd &msg);
      void callbackClearFault(const add_sugv_msgs::msg::ClearFaultCmd &msg);
      void callbackRemoteCmd(const add_sugv_msgs::msg::TwistCmd &msg);
      void callbackGripperCmd(const add_sugv_msgs::msg::GripperCmd &msg);
      void callbackJointCmd(const add_sugv_msgs::msg::JointCmd &msg);

      // Robot States
      void callbackState(const kinova_msgs::msg::State &msg);                      
      void callbackStatus(const kinova_msgs::msg::Status &msg);                    
      void callbackPresetList(const kinova_msgs::msg::ActionList &msg);
      void callbackProtectionZoneList(const kinova_msgs::msg::ProtectionZoneList &msg);
      void callbackManipulatorStatus();                                           

      // Setting Commands
      void callbackPresetTargetCmd(const add_sugv_msgs::msg::PresetCmd &msg);
      void callbackPresetMaker(const add_sugv_msgs::msg::PresetCmd &msg);
      void callbackPresetRemover(const add_sugv_msgs::msg::PresetCmd &msg);
      void callbackSafetyChecker(const std_msgs::msg::Bool &msg);                  
      void callbackPresetInfo(const kinova_msgs::msg::ActionInfoArray &msg);
      void callbackProtectionZoneInfo(const kinova_msgs::msg::ProtectionZoneInfoArray &msg);
      void callbackSetProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg);
      void callbackDeleteProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg); 
      void callbackUpdateProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg); 

      // Utility Function. Get Namespace from PNP Manager
      std::string getNamespace();

      // Member Variables
      std::shared_ptr<MyAppRunnable> runnable_;
      std::shared_ptr<spdlog::logger> logger_;

      // ROS Topic Names
      std::string pub_emergency_stop_, pub_clear_fault_, pub_remote_cmd_, pub_gripper_cmd_, pub_joint_cmd_, pub_preset_target_cmd_;
      std::string sub_emergency_stop_, sub_clear_fault_, sub_remote_cmd_, sub_gripper_cmd_, sub_joint_cmd_, sub_preset_target_cmd_;      
      std::string pub_preset_maker_, pub_preset_remover_, pub_safety_checker_, pub_preset_info_, pub_protection_zone_info_, pub_set_protection_zone_, pub_delete_protection_zone_, pub_update_protection_zone_;
      std::string sub_preset_maker_, sub_preset_remover_, sub_safety_checker_, sub_preset_info_, sub_protection_zone_info_, sub_set_protection_zone_, sub_delete_protection_zone_, sub_update_protection_zone_;
      std::string sub_preset_list_, sub_protection_zone_list_, sub_state_, sub_status_;
      std::string pub_manipulator_status_;  
      std::string sub_manipulator_status_;

      // Robot State Variables
      kinova_msgs::msg::State last_state_;   
      kinova_msgs::msg::Status last_status_; 
      std::vector<std::string> last_preset_list_;
      std::vector<std::string> last_protection_zone_list_;

      bool state_received_ = false;
      bool status_received_ = false;
      bool e_stop_active_ = false;
      bool preset_list_received_ = false;
      bool protection_zone_list_received_ = false;

      
      // ROS Client for Diagnostic Service
      rclcpp::Client<add_sugv_msgs::srv::RegisterService>::SharedPtr diagnostic_client_;
      add_sugv_msgs::srv::RegisterService::Request::SharedPtr diagnostic_request_;
      std::string namespace_;
    };

  }  // namespace ROS
}  // namespace acelab

#endif  // INCLUDE_MY_PROJECT_RUNNER_ROS_RUNNER_HPP_
