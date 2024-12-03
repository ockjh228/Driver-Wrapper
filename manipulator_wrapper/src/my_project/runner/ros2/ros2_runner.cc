/**
 * @file ros_runnable.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#include <tf2/LinearMath/Quaternion.h>
#include <iostream>
#include <cmath>


#include "my_project/runner/ros2/ros2_runner.hpp"
#include "my_project/base/base_runnable.hpp"

#include "add_sugv_msgs/msg/emergency_stop_cmd.hpp"
#include "add_sugv_msgs/msg/clear_fault_cmd.hpp"
#include "add_sugv_msgs/msg/gripper_cmd.hpp"
#include "add_sugv_msgs/msg/manipulator_status.hpp"
#include "add_sugv_msgs/msg/safety_check.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "control_msgs/msg/gripper_command.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp" 

// #define PUB_QUEUE_SIZE 1
// #define SUB_QUEUE_SIZE 1

#define PUB_QUEUE_SIZE 10
#define SUB_QUEUE_SIZE 10

#define REMOTE "remote"
#define KIT "mission"
#define INTEGRATED "integrated"

namespace acelab {
  namespace ROS {
    Runner::Runner(std::shared_ptr<MyAppRunnable> runnable,
                  int argc,
                  char **argv,
                  std::unordered_map<std::string, int> domain_ids)
        : ROS2::Runner(std::dynamic_pointer_cast<BaseRunnable>(runnable),
                      argc,
                      argv,
                      "manipulator_wrapper" + getCpuUid(),
                      domain_ids),
          runnable_(runnable),
          e_stop_active_(false){
      logger_ = runnable_->getLoggerHandler()->getLogger();
    }

    Runner::~Runner() {
      // -- To Kinova Driver --
      ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::EmergencyStopCmd>(pub_emergency_stop_);
      ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::ClearFaultCmd>(pub_clear_fault_);
      ROS2::Node::getInstance().unregisterPublisher<geometry_msgs::msg::Twist>(pub_remote_cmd_);
      ROS2::Node::getInstance().unregisterPublisher<trajectory_msgs::msg::JointTrajectory>(pub_joint_cmd_);
      ROS2::Node::getInstance().unregisterPublisher<control_msgs::msg::GripperCommand>(pub_gripper_cmd_);
      ROS2::Node::getInstance().unregisterPublisher<std_msgs::msg::String>(pub_preset_target_cmd_);
      ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::CreateAction>(pub_preset_maker_);
      ROS2::Node::getInstance().unregisterPublisher<std_msgs::msg::String>(pub_preset_remover_);
      ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::ProtectionZone>(pub_set_protection_zone_);
      ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::ProtectionZone>(pub_delete_protection_zone_);
      ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::ProtectionZone>(pub_update_protection_zone_);

      // -- To Remote Controller --
      ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::SafetyCheck>(pub_safety_checker_);
      ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::PresetInfoArray>(pub_preset_info_);
      ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::ProtectionZoneInfoArray>(pub_protection_zone_info_);
      ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::ManipulatorStatus>(pub_manipulator_status_);

      // -- From Remote Controller --
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::EmergencyStopCmd>(sub_emergency_stop_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ClearFaultCmd>(sub_clear_fault_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::TwistCmd>(sub_remote_cmd_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::JointCmd>(sub_joint_cmd_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::GripperCmd>(sub_gripper_cmd_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_target_cmd_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_maker_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_remover_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_set_protection_zone_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_delete_protection_zone_);
      ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_update_protection_zone_);

      // -- From Kinova Driver --
      ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::State>(sub_state_);
      ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::Status>(sub_status_);
      ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ActionList>(sub_preset_list_);
      ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ProtectionZoneList>(sub_protection_zone_list_);
      ROS2::Node::getInstance().unregisterSubscriber<std_msgs::msg::Bool>(sub_safety_checker_);
      ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ActionInfoArray>(sub_preset_info_);
      ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ProtectionZoneInfoArray>(sub_protection_zone_info_);      
    }

    // Request Service to PNP Manager to get namespace for Manipulator.
    std::string Runner::getNamespace() {
      std::string ros_namespace = "";

      if (runnable_->getParameterManager()->get<bool>(
            {"system", "pnp_manager_enable"}, false) == false) {
        return ros_namespace;
      }

      std::string register_service_name =
        runnable_->getParameterManager()->get<std::string>(
          {"system", "service", "register_service"}, "");
      std::cout << "register_service_name : " << register_service_name << std::endl;
      diagnostic_client_ = this->getNodeHandler(INTEGRATED)
                             ->create_client<add_sugv_msgs::srv::RegisterService>(
                               register_service_name);
      std::cout << "waiting service until available..." << std::endl;
      while (!diagnostic_client_->wait_for_service(std::chrono::seconds(1))) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      std::cout << "service is available" << std::endl;
      diagnostic_request_ =
        std::make_shared<add_sugv_msgs::srv::RegisterService::Request>();
      diagnostic_request_->type =
        runnable_->getParameterManager()->get<std::string>({"system", "device_id"},
                                                           "");
                                                          
      diagnostic_request_->uuid = getCpuUid();
      auto result = diagnostic_client_->async_send_request(diagnostic_request_);

      if (rclcpp::spin_until_future_complete(this->getNodeHandler(INTEGRATED),
                                             result) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        auto response = result.get();

        ros_namespace = response->topic_namespace;
        std::cout << "namespace is " << ros_namespace << std::endl;
        return ros_namespace;
      }
      return "";
    }

    void Runner::init() {
      logger_->info("ROS Runner init");

      /**
       * register publisher and subscriber
       * they are managed by topic name like pub1_ and sub1_
       */

      namespace_ = getNamespace();
      
      /*
      Publishers
      */
      pub_emergency_stop_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "emergency_stop"}, "default_emergency_stop");
      ROS2::Node::getInstance().registerPublisher<std_msgs::msg::Bool>(pub_emergency_stop_, PUB_QUEUE_SIZE, KIT);
      
      pub_clear_fault_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "clear_fault"}, "default_clear_stop");
      ROS2::Node::getInstance().registerPublisher<std_msgs::msg::Bool>(pub_clear_fault_, PUB_QUEUE_SIZE, KIT);

      pub_remote_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "remote_cmd"}, "default_remote_cmd");
      ROS2::Node::getInstance().registerPublisher<geometry_msgs::msg::Twist>(pub_remote_cmd_, 1, KIT);
      
      pub_joint_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "joint_cmd"}, "default_joint_cmd");
      ROS2::Node::getInstance().registerPublisher<trajectory_msgs::msg::JointTrajectory>(pub_joint_cmd_, PUB_QUEUE_SIZE, KIT);

      pub_preset_target_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "preset_target"}, "default_preset_target");
      ROS2::Node::getInstance().registerPublisher<std_msgs::msg::String>(pub_preset_target_cmd_, PUB_QUEUE_SIZE, KIT);
      
      pub_gripper_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "gripper_cmd"}, "default_gripper_cmd");
      ROS2::Node::getInstance().registerPublisher<control_msgs::msg::GripperCommand>(pub_gripper_cmd_, PUB_QUEUE_SIZE, KIT);

      pub_preset_maker_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "create_preset"}, "default_create_preset");
      ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::CreateAction>(pub_preset_maker_, PUB_QUEUE_SIZE, KIT);

      pub_preset_remover_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "delete_preset"}, "default_delete_preset");
      ROS2::Node::getInstance().registerPublisher<std_msgs::msg::String>(pub_preset_remover_, PUB_QUEUE_SIZE, KIT);

      pub_set_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "create_protection_zone"}, "/protection_zone");
      ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::ProtectionZone>(pub_set_protection_zone_, PUB_QUEUE_SIZE, KIT);

      pub_delete_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "delete_protection_zone"}, "/protection_zone");
      ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::ProtectionZone>(pub_delete_protection_zone_, PUB_QUEUE_SIZE, KIT);

      pub_update_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "update_protection_zone"}, "/protection_zone");
      ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::ProtectionZone>(pub_update_protection_zone_, PUB_QUEUE_SIZE, KIT);



      pub_safety_checker_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "safety_check"}, "default_safety_check");
        if(pub_safety_checker_.front() != '/'){
          pub_safety_checker_ = "/" + pub_safety_checker_;
        }
        pub_safety_checker_ = namespace_ + pub_safety_checker_;
      ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::SafetyCheck>(pub_safety_checker_, PUB_QUEUE_SIZE, INTEGRATED);

      pub_preset_info_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "preset_info"}, "default_preset_info");
        if(pub_preset_info_.front() != '/'){
          pub_preset_info_ = "/" + pub_preset_info_;
        }
        pub_preset_info_ = namespace_ + pub_preset_info_;
      ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::PresetInfoArray>(pub_preset_info_, PUB_QUEUE_SIZE, INTEGRATED);

      pub_protection_zone_info_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "protection_zone_info"}, "default_protection_zone_info");
        if(pub_protection_zone_info_.front() != '/'){
          pub_protection_zone_info_ = "/" + pub_protection_zone_info_;
        }
        pub_protection_zone_info_ = namespace_ + pub_protection_zone_info_;
      ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::ProtectionZoneInfoArray>(pub_protection_zone_info_, PUB_QUEUE_SIZE, INTEGRATED);

      pub_manipulator_status_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "manipulator_status"}, "default_manipulator_status");
        if(pub_manipulator_status_.front() != '/'){
          pub_manipulator_status_ = "/" + pub_manipulator_status_;
        }
        pub_manipulator_status_ = namespace_ + pub_manipulator_status_;
      ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::ManipulatorStatus>(pub_manipulator_status_, PUB_QUEUE_SIZE, INTEGRATED);

      
      /*
      Subscribers
      */

      sub_emergency_stop_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "emergency_stop"});    
        if(sub_emergency_stop_.front() != '/'){
          sub_emergency_stop_ = "/" + sub_emergency_stop_;
        }
        sub_emergency_stop_ = namespace_ + sub_emergency_stop_;
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::EmergencyStopCmd>(sub_emergency_stop_, SUB_QUEUE_SIZE, &Runner::callbackEmergencyStop, this, INTEGRATED);

      sub_clear_fault_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "clear_fault"});    
         if(sub_clear_fault_.front() != '/'){
          sub_clear_fault_ = "/" + sub_clear_fault_;
        }
        sub_clear_fault_ = namespace_ + sub_clear_fault_;  
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ClearFaultCmd>(sub_clear_fault_, SUB_QUEUE_SIZE, &Runner::callbackClearFault, this, INTEGRATED);

      sub_remote_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "remote_cmd"});    
        if(sub_remote_cmd_.front() != '/'){
          sub_remote_cmd_ = "/" + sub_remote_cmd_;
        }
        sub_remote_cmd_ = namespace_ + sub_remote_cmd_;  
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::TwistCmd>(sub_remote_cmd_, SUB_QUEUE_SIZE, &Runner::callbackRemoteCmd, this, INTEGRATED);

      sub_gripper_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "gripper_cmd"});    
        if(sub_gripper_cmd_.front() != '/'){
          sub_gripper_cmd_ = "/" + sub_gripper_cmd_;
        }
        sub_gripper_cmd_ = namespace_ + sub_gripper_cmd_;  
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::GripperCmd>(sub_gripper_cmd_, SUB_QUEUE_SIZE, &Runner::callbackGripperCmd, this, INTEGRATED);

      sub_joint_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "joint_cmd"});  
        if(sub_joint_cmd_.front() != '/'){
          sub_joint_cmd_ = "/" + sub_joint_cmd_;
        }
        sub_joint_cmd_ = namespace_ + sub_joint_cmd_;  
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::JointCmd>(sub_joint_cmd_, SUB_QUEUE_SIZE, &Runner::callbackJointCmd, this, INTEGRATED);

      sub_preset_target_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "preset_target"});    
        if(sub_preset_target_cmd_.front() != '/'){
          sub_preset_target_cmd_ = "/" + sub_preset_target_cmd_;
        }
        sub_preset_target_cmd_ = namespace_ + sub_preset_target_cmd_; 
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_target_cmd_, SUB_QUEUE_SIZE, &Runner::callbackPresetTargetCmd, this, INTEGRATED);

      sub_preset_maker_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "create_preset"});
        if(sub_preset_maker_.front() != '/'){
          sub_preset_maker_ = "/" + sub_preset_maker_;
        }
        sub_preset_maker_ = namespace_ + sub_preset_maker_;
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_maker_, SUB_QUEUE_SIZE, &Runner::callbackPresetMaker, this, INTEGRATED);

      sub_preset_remover_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "delete_preset"});
        if(sub_preset_remover_.front() != '/'){
          sub_preset_remover_ = "/" + sub_preset_remover_;
        }
        sub_preset_remover_ = namespace_ + sub_preset_remover_;
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_remover_, SUB_QUEUE_SIZE, &Runner::callbackPresetRemover, this, INTEGRATED);


      sub_set_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "create_protection_zone"});
        if(sub_set_protection_zone_.front() != '/'){
          sub_set_protection_zone_ = "/" + sub_set_protection_zone_;
        }
        sub_set_protection_zone_ = namespace_ + sub_set_protection_zone_;
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_set_protection_zone_, SUB_QUEUE_SIZE, &Runner::callbackSetProtectionZone, this, INTEGRATED);

      sub_delete_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "delete_protection_zone"});
        if(sub_delete_protection_zone_.front() != '/'){
          sub_delete_protection_zone_ = "/" + sub_delete_protection_zone_;
        }
        sub_delete_protection_zone_ = namespace_ + sub_delete_protection_zone_;
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_delete_protection_zone_, SUB_QUEUE_SIZE, &Runner::callbackDeleteProtectionZone, this, INTEGRATED);

      sub_update_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "update_protection_zone"});
        if(sub_update_protection_zone_.front() != '/'){
          sub_update_protection_zone_ = "/" + sub_update_protection_zone_;
        }
        sub_update_protection_zone_ = namespace_ + sub_update_protection_zone_;
      ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_update_protection_zone_, SUB_QUEUE_SIZE, &Runner::callbackUpdateProtectionZone, this, INTEGRATED);




      sub_state_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "state"});
      ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::State>(sub_state_, SUB_QUEUE_SIZE, &Runner::callbackState, this, KIT);

      sub_status_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "status"});
      ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::Status>(sub_status_, SUB_QUEUE_SIZE, &Runner::callbackStatus, this, KIT);

      sub_preset_list_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "action_list"});
      ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ActionList>(sub_preset_list_, SUB_QUEUE_SIZE, &Runner::callbackPresetList, this, KIT);

      sub_protection_zone_list_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "protection_zone_list"});
      ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ProtectionZoneList>(sub_protection_zone_list_, SUB_QUEUE_SIZE, &Runner::callbackProtectionZoneList, this, KIT);

      sub_preset_info_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "preset_info"});
      ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ActionInfoArray>(sub_preset_info_, SUB_QUEUE_SIZE, &Runner::callbackPresetInfo, this, KIT);

      sub_protection_zone_info_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "protection_zone_info"});
      ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ProtectionZoneInfoArray>(sub_protection_zone_info_, SUB_QUEUE_SIZE, &Runner::callbackProtectionZoneInfo, this, KIT);

      sub_safety_checker_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "safety_check"});
      ROS2::Node::getInstance().registerSubscriber<std_msgs::msg::Bool>(sub_safety_checker_, SUB_QUEUE_SIZE, &Runner::callbackSafetyChecker, this, KIT);
      
      /*  
        // clang-format on
      */
    }

    void Runner::start() {
      /**
       * get message from topic
       * if message is not received, it returns std::nullopt
       */
      // auto sub_msg =
      //   ROS2::Node::getInstance().getSubscribeMessage<std_msgs::msg::Int64>(sub2_);
      // if (sub_msg) {
      //   runnable_->setC(convert<std_msgs::msg::Int64, int64_t>(*sub_msg));
      // }
    }

    void Runner::end() {
      /**
       * publish message to topic
       */
      // ROS2::Node::getInstance().publishMessage<std_msgs::msg::Int64>(
      //   pub2_, convert<int64_t, std_msgs::msg::Int64>(runnable_->getC()));
    }

    void Runner::diagnose() { logger_->info("ROS Runner diagnose"); }

    void Runner::callbackEmergencyStop(const add_sugv_msgs::msg::EmergencyStopCmd &msg) {
      e_stop_active_ = true;

      auto converted_msg = Interface::convert<add_sugv_msgs::msg::EmergencyStopCmd, std_msgs::msg::Bool>(msg);
      ROS2::Node::getInstance().publishMessage<std_msgs::msg::Bool>(pub_emergency_stop_, converted_msg);
      logger_->info("Published converted EmergencyStopCmd to Empty");
    }

    void Runner::callbackClearFault(const add_sugv_msgs::msg::ClearFaultCmd &msg) {
      e_stop_active_ = false;

      auto converted_msg = Interface::convert<add_sugv_msgs::msg::ClearFaultCmd, std_msgs::msg::Bool>(msg);
      ROS2::Node::getInstance().publishMessage<std_msgs::msg::Bool>(pub_clear_fault_, converted_msg);
      logger_->info("Published converted ClearFaultCmd to Empty");
    }

    void Runner::callbackRemoteCmd(const add_sugv_msgs::msg::TwistCmd &msg) {
      if (e_stop_active_) { 
        logger_->warn("E-stop is active. Ignoring RemoteCmd.");
        return;
      }

      auto converted_msg = Interface::convert<add_sugv_msgs::msg::TwistCmd, geometry_msgs::msg::Twist>(msg);
      ROS2::Node::getInstance().publishMessage<geometry_msgs::msg::Twist>(pub_remote_cmd_, converted_msg);
      logger_->info("Published converted RemoteCmd to Twist");
    }

    void Runner::callbackGripperCmd(const add_sugv_msgs::msg::GripperCmd &msg) {
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::GripperCmd, control_msgs::msg::GripperCommand>(msg);
      ROS2::Node::getInstance().publishMessage<control_msgs::msg::GripperCommand>(pub_gripper_cmd_, converted_msg);
      logger_->info("Published converted GripperCmd to GripperCommand");
    }

    void Runner::callbackJointCmd(const add_sugv_msgs::msg::JointCmd &msg) {
      if (e_stop_active_) { 
        logger_->warn("E-stop is active. Ignoring RemoteJointCmd");
        return;
      }
      try {
        auto converted_msg = Interface::convert<add_sugv_msgs::msg::JointCmd, trajectory_msgs::msg::JointTrajectory>(msg);

        ROS2::Node::getInstance().publishMessage<trajectory_msgs::msg::JointTrajectory>(pub_joint_cmd_, converted_msg);
      }
      catch (const std::bad_any_cast& e) {
        // bad_any_cast occured
        logger_->error("Failed to convert RemoteJointCmd to JointTrajectory: bad_any_cast - {}", e.what());
      } 
      catch (const std::exception& e) {
        // Other Exceptions....
        logger_->error("Failed to convert RemoteJointCmd to JointTrajectory: {}", e.what());
      }
    }

    void Runner::callbackState(const kinova_msgs::msg::State &msg) {
      last_state_ = msg;
      state_received_ = true;
      callbackManipulatorStatus(); 
    }

    void Runner::callbackStatus(const kinova_msgs::msg::Status &msg) {
      last_status_ = msg;
      status_received_ = true;
      callbackManipulatorStatus(); 
    }

    void Runner::callbackPresetList(const kinova_msgs::msg::ActionList &msg){
      last_preset_list_ = msg.action_list;
      preset_list_received_ = true;
      callbackManipulatorStatus();
    }

    void Runner::callbackProtectionZoneList(const kinova_msgs::msg::ProtectionZoneList &msg){
      last_protection_zone_list_ = msg.protection_zone_list;
      protection_zone_list_received_ = true;
      callbackManipulatorStatus();
    }

    void Runner::callbackManipulatorStatus() {
      if (state_received_ && status_received_ && preset_list_received_ && protection_zone_list_received_) {
        acelab::ManipulatorStatus manipulator_status_struct;
        manipulator_status_struct.state = last_state_;    
        manipulator_status_struct.status = last_status_;
        manipulator_status_struct.action_list = last_preset_list_;
        manipulator_status_struct.protection_zone_list = last_protection_zone_list_;

        add_sugv_msgs::msg::ManipulatorStatus manipulator_status_msg = 
            Interface::convert<acelab::ManipulatorStatus, add_sugv_msgs::msg::ManipulatorStatus>(manipulator_status_struct);

        ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::ManipulatorStatus>(pub_manipulator_status_, manipulator_status_msg);

        state_received_ = false;
        status_received_ = false;
        preset_list_received_ = false;
        protection_zone_list_received_ = false;
      }
    }

    void Runner::callbackPresetTargetCmd(const add_sugv_msgs::msg::PresetCmd &msg){
      if (e_stop_active_) { 
        logger_->warn("E-stop is active. Ignoring Cmd.");
        return;
      }
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::PresetCmd, std_msgs::msg::String>(msg);
      ROS2::Node::getInstance().publishMessage<std_msgs::msg::String>(pub_preset_target_cmd_, converted_msg);
    }

    void Runner::callbackPresetMaker(const add_sugv_msgs::msg::PresetCmd &msg){
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::PresetCmd, kinova_msgs::msg::CreateAction>(msg);
      ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::CreateAction>(pub_preset_maker_, converted_msg);
    }

    void Runner::callbackPresetRemover(const add_sugv_msgs::msg::PresetCmd &msg){
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::PresetCmd, std_msgs::msg::String>(msg);
      ROS2::Node::getInstance().publishMessage<std_msgs::msg::String>(pub_preset_remover_, converted_msg);
    }

    void Runner::callbackSafetyChecker(const std_msgs::msg::Bool &msg){
      auto converted_msg = Interface::convert<std_msgs::msg::Bool, add_sugv_msgs::msg::SafetyCheck>(msg);
      ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::SafetyCheck>(pub_safety_checker_,converted_msg);
    }

    void Runner::callbackPresetInfo(const kinova_msgs::msg::ActionInfoArray &msg){
      auto converted_msg = Interface::convert<kinova_msgs::msg::ActionInfoArray, add_sugv_msgs::msg::PresetInfoArray>(msg);
      ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::PresetInfoArray>(pub_preset_info_, converted_msg);
    }

    void Runner::callbackProtectionZoneInfo(const kinova_msgs::msg::ProtectionZoneInfoArray &msg){
      auto converted_msg = Interface::convert<kinova_msgs::msg::ProtectionZoneInfoArray, add_sugv_msgs::msg::ProtectionZoneInfoArray>(msg);
      ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::ProtectionZoneInfoArray>(pub_protection_zone_info_, converted_msg);
    }

    void Runner::callbackSetProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg) {
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::ProtectionZoneCmd, kinova_msgs::msg::ProtectionZone>(msg);

      ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::ProtectionZone>(pub_set_protection_zone_, converted_msg);
    }

    void Runner::callbackDeleteProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg){
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::ProtectionZoneCmd, kinova_msgs::msg::ProtectionZone>(msg);

      ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::ProtectionZone>(pub_delete_protection_zone_, converted_msg);
    }

    void Runner::callbackUpdateProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg){
      auto converted_msg = Interface::convert<add_sugv_msgs::msg::ProtectionZoneCmd, kinova_msgs::msg::ProtectionZone>(msg);

      ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::ProtectionZone>(pub_update_protection_zone_, converted_msg);
    }

  }  // namespace ROS
}  // namespace acelab




/****************************************************************/
/****************************************************************/
/****************************************************************/
/****************************************************************/
/****************************************************************/



// /*
//   Only For "TEST". Set Constant Single Domain -> Integrated
// */

// /**
//  * @file ros_runnable.cc
//  * @author your name (you@domain.com)
//  * @brief
//  * @version 0.1
//  * @date 2022-11-15
//  *
//  * @copyright Copyright (c)ACELAB 2022
//  *
//  */

// #include <tf2/LinearMath/Quaternion.h>
// #include <iostream>
// #include <cmath>


// #include "my_project/runner/ros2/ros2_runner.hpp"
// #include "my_project/base/base_runnable.hpp"

// #include "add_sugv_msgs/msg/emergency_stop_cmd.hpp"
// #include "add_sugv_msgs/msg/clear_fault_cmd.hpp"
// #include "add_sugv_msgs/msg/gripper_cmd.hpp"
// #include "add_sugv_msgs/msg/manipulator_status.hpp"
// #include "add_sugv_msgs/msg/safety_check.hpp"

// #include "std_msgs/msg/empty.hpp"
// #include "std_msgs/msg/bool.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "control_msgs/msg/gripper_command.hpp"
// #include "trajectory_msgs/msg/joint_trajectory.hpp" 

// // #define PUB_QUEUE_SIZE 1
// // #define SUB_QUEUE_SIZE 1

// #define PUB_QUEUE_SIZE 10
// #define SUB_QUEUE_SIZE 10

// #define REMOTE "remote"
// #define KIT "mission"
// #define INTEGRATED "integrated"

// namespace acelab {
//   namespace ROS {
//     Runner::Runner(std::shared_ptr<MyAppRunnable> runnable,
//                   int argc,
//                   char **argv,
//                   std::unordered_map<std::string, int> domain_ids)
//         : ROS2::Runner(std::dynamic_pointer_cast<BaseRunnable>(runnable),
//                       argc,
//                       argv,
//                       "manipulator_wrapper" + getCpuUid(),
//                       domain_ids),
//           runnable_(runnable),
//           e_stop_active_(false){
//       logger_ = runnable_->getLoggerHandler()->getLogger();
//     }

//     Runner::~Runner() {
//       // -- To Kinova Driver --
//       ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::EmergencyStopCmd>(pub_emergency_stop_);
//       ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::ClearFaultCmd>(pub_clear_fault_);
//       ROS2::Node::getInstance().unregisterPublisher<geometry_msgs::msg::Twist>(pub_remote_cmd_);
//       ROS2::Node::getInstance().unregisterPublisher<trajectory_msgs::msg::JointTrajectory>(pub_joint_cmd_);
//       ROS2::Node::getInstance().unregisterPublisher<control_msgs::msg::GripperCommand>(pub_gripper_cmd_);
//       ROS2::Node::getInstance().unregisterPublisher<std_msgs::msg::String>(pub_preset_target_cmd_);
//       ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::CreateAction>(pub_preset_maker_);
//       ROS2::Node::getInstance().unregisterPublisher<std_msgs::msg::String>(pub_preset_remover_);
//       ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::ProtectionZone>(pub_set_protection_zone_);
//       ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::ProtectionZone>(pub_delete_protection_zone_);
//       ROS2::Node::getInstance().unregisterPublisher<kinova_msgs::msg::ProtectionZone>(pub_update_protection_zone_);

//       // -- To Remote Controller --
//       ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::SafetyCheck>(pub_safety_checker_);
//       ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::PresetInfoArray>(pub_preset_info_);
//       ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::ProtectionZoneInfoArray>(pub_protection_zone_info_);
//       ROS2::Node::getInstance().unregisterPublisher<add_sugv_msgs::msg::ManipulatorStatus>(pub_manipulator_status_);

//       // -- From Remote Controller --
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::EmergencyStopCmd>(sub_emergency_stop_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ClearFaultCmd>(sub_clear_fault_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::TwistCmd>(sub_remote_cmd_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::JointCmd>(sub_joint_cmd_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::GripperCmd>(sub_gripper_cmd_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_target_cmd_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_maker_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_remover_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_set_protection_zone_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_delete_protection_zone_);
//       ROS2::Node::getInstance().unregisterSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_update_protection_zone_);

//       // -- From Kinova Driver --
//       ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::State>(sub_state_);
//       ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::Status>(sub_status_);
//       ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ActionList>(sub_preset_list_);
//       ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ProtectionZoneList>(sub_protection_zone_list_);
//       ROS2::Node::getInstance().unregisterSubscriber<std_msgs::msg::Bool>(sub_safety_checker_);
//       ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ActionInfoArray>(sub_preset_info_);
//       ROS2::Node::getInstance().unregisterSubscriber<kinova_msgs::msg::ProtectionZoneInfoArray>(sub_protection_zone_info_);      
//     }

//     // std::string Runner::getNamespace() {
//     //   std::string ros_namespace = "";

//     //   if (runnable_->getParameterManager()->get<bool>(
//     //         {"system", "pnp_manager_enable"}, false) == false) {
//     //     return ros_namespace;
//     //   }

//     //   std::string register_service_name =
//     //     runnable_->getParameterManager()->get<std::string>(
//     //       {"system", "service", "register_service"}, "");
//     //   std::cout << "register_service_name : " << register_service_name << std::endl;
//     //   diagnostic_client_ = this->getNodeHandler(INTEGRATED)
//     //                          ->create_client<add_sugv_msgs::srv::RegisterService>(
//     //                            register_service_name);
//     //   std::cout << "waiting service until available..." << std::endl;
//     //   while (!diagnostic_client_->wait_for_service(std::chrono::seconds(1))) {
//     //     std::this_thread::sleep_for(std::chrono::milliseconds(100));
//     //   }
//     //   std::cout << "service is available" << std::endl;
//     //   diagnostic_request_ =
//     //     std::make_shared<add_sugv_msgs::srv::RegisterService::Request>();
//     //   diagnostic_request_->type =
//     //     runnable_->getParameterManager()->get<std::string>({"system", "device_id"},
//     //                                                        "");
                                                          
//     //   diagnostic_request_->uuid = getCpuUid();
//     //   auto result = diagnostic_client_->async_send_request(diagnostic_request_);

//     //   if (rclcpp::spin_until_future_complete(this->getNodeHandler(INTEGRATED),
//     //                                          result) ==
//     //       rclcpp::FutureReturnCode::SUCCESS) {
//     //     auto response = result.get();

//     //     ros_namespace = response->topic_namespace;
//     //     std::cout << "namespace is " << ros_namespace << std::endl;
//     //     return ros_namespace;
//     //   }
//     //   return "";
//     // }

//     void Runner::init() {
//       logger_->info("ROS Runner init");

//       /**
//        * register publisher and subscriber
//        * they are managed by topic name like pub1_ and sub1_
//        */

//       // namespace_ = getNamespace();
      
//       /*
//       Publishers
//       */
//       pub_emergency_stop_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "emergency_stop"}, "default_emergency_stop");
//       ROS2::Node::getInstance().registerPublisher<std_msgs::msg::Bool>(pub_emergency_stop_, PUB_QUEUE_SIZE, INTEGRATED);
      
//       pub_clear_fault_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "clear_fault"}, "default_clear_stop");
//       ROS2::Node::getInstance().registerPublisher<std_msgs::msg::Bool>(pub_clear_fault_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_remote_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "remote_cmd"}, "default_remote_cmd");
//       ROS2::Node::getInstance().registerPublisher<geometry_msgs::msg::Twist>(pub_remote_cmd_, 1, INTEGRATED);
      
//       pub_joint_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "joint_cmd"}, "default_joint_cmd");
//       ROS2::Node::getInstance().registerPublisher<trajectory_msgs::msg::JointTrajectory>(pub_joint_cmd_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_preset_target_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "preset_target"}, "default_preset_target");
//       ROS2::Node::getInstance().registerPublisher<std_msgs::msg::String>(pub_preset_target_cmd_, PUB_QUEUE_SIZE, INTEGRATED);
      
//       pub_gripper_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "gripper_cmd"}, "default_gripper_cmd");
//       ROS2::Node::getInstance().registerPublisher<control_msgs::msg::GripperCommand>(pub_gripper_cmd_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_preset_maker_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "create_preset"}, "default_create_preset");
//       ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::CreateAction>(pub_preset_maker_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_preset_remover_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "delete_preset"}, "default_delete_preset");
//       ROS2::Node::getInstance().registerPublisher<std_msgs::msg::String>(pub_preset_remover_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_set_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "create_protection_zone"}, "/protection_zone");
//       ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::ProtectionZone>(pub_set_protection_zone_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_delete_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "delete_protection_zone"}, "/protection_zone");
//       ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::ProtectionZone>(pub_delete_protection_zone_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_update_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "update_protection_zone"}, "/protection_zone");
//       ROS2::Node::getInstance().registerPublisher<kinova_msgs::msg::ProtectionZone>(pub_update_protection_zone_, PUB_QUEUE_SIZE, INTEGRATED);



//       pub_safety_checker_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "safety_check"}, "default_safety_check");
//         // if(pub_safety_checker_.front() != '/'){
//         //   pub_safety_checker_ = "/" + pub_safety_checker_;
//         // }
//         // pub_safety_checker_ = namespace_ + pub_safety_checker_;
//       ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::SafetyCheck>(pub_safety_checker_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_preset_info_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "preset_info"}, "default_preset_info");
//         // if(pub_preset_info_.front() != '/'){
//         //   pub_preset_info_ = "/" + pub_preset_info_;
//         // }
//         // pub_preset_info_ = namespace_ + pub_preset_info_;
//       ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::PresetInfoArray>(pub_preset_info_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_protection_zone_info_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "protection_zone_info"}, "default_protection_zone_info");
//         // if(pub_protection_zone_info_.front() != '/'){
//         //   pub_protection_zone_info_ = "/" + pub_protection_zone_info_;
//         // }
//         // pub_protection_zone_info_ = namespace_ + pub_protection_zone_info_;
//       ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::ProtectionZoneInfoArray>(pub_protection_zone_info_, PUB_QUEUE_SIZE, INTEGRATED);

//       pub_manipulator_status_ = runnable_->getParameterManager()->get<std::string>({"system", "publish", "manipulator_status"}, "default_manipulator_status");
//         // if(pub_manipulator_status_.front() != '/'){
//         //   pub_manipulator_status_ = "/" + pub_manipulator_status_;
//         // }
//         // pub_manipulator_status_ = namespace_ + pub_manipulator_status_;
//       ROS2::Node::getInstance().registerPublisher<add_sugv_msgs::msg::ManipulatorStatus>(pub_manipulator_status_, PUB_QUEUE_SIZE, INTEGRATED);

      
//       /*
//       Subscribers
//       */

//       sub_emergency_stop_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "emergency_stop"});    
//         // if(sub_emergency_stop_.front() != '/'){
//         //   sub_emergency_stop_ = "/" + sub_emergency_stop_;
//         // }
//         // sub_emergency_stop_ = namespace_ + sub_emergency_stop_;
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::EmergencyStopCmd>(sub_emergency_stop_, SUB_QUEUE_SIZE, &Runner::callbackEmergencyStop, this, INTEGRATED);

//       sub_clear_fault_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "clear_fault"});    
//         //  if(sub_clear_fault_.front() != '/'){
//         //   sub_clear_fault_ = "/" + sub_clear_fault_;
//         // }
//         // sub_clear_fault_ = namespace_ + sub_clear_fault_;  
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ClearFaultCmd>(sub_clear_fault_, SUB_QUEUE_SIZE, &Runner::callbackClearFault, this, INTEGRATED);

//       sub_remote_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "remote_cmd"});    
//         // if(sub_remote_cmd_.front() != '/'){
//         //   sub_remote_cmd_ = "/" + sub_remote_cmd_;
//         // }
//         // sub_remote_cmd_ = namespace_ + sub_remote_cmd_;  
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::TwistCmd>(sub_remote_cmd_, SUB_QUEUE_SIZE, &Runner::callbackRemoteCmd, this, INTEGRATED);

//       sub_gripper_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "gripper_cmd"});    
//         // if(sub_gripper_cmd_.front() != '/'){
//         //   sub_gripper_cmd_ = "/" + sub_gripper_cmd_;
//         // }
//         // sub_gripper_cmd_ = namespace_ + sub_gripper_cmd_;  
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::GripperCmd>(sub_gripper_cmd_, SUB_QUEUE_SIZE, &Runner::callbackGripperCmd, this, INTEGRATED);

//       sub_joint_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "joint_cmd"});  
//         // if(sub_joint_cmd_.front() != '/'){
//         //   sub_joint_cmd_ = "/" + sub_joint_cmd_;
//         // }
//         // sub_joint_cmd_ = namespace_ + sub_joint_cmd_;  
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::JointCmd>(sub_joint_cmd_, SUB_QUEUE_SIZE, &Runner::callbackJointCmd, this, INTEGRATED);

//       sub_preset_target_cmd_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "preset_target"});    
//         // if(sub_preset_target_cmd_.front() != '/'){
//         //   sub_preset_target_cmd_ = "/" + sub_preset_target_cmd_;
//         // }
//         // sub_preset_target_cmd_ = namespace_ + sub_preset_target_cmd_; 
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_target_cmd_, SUB_QUEUE_SIZE, &Runner::callbackPresetTargetCmd, this, INTEGRATED);

//       sub_preset_maker_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "create_preset"});
//         // if(sub_preset_maker_.front() != '/'){
//         //   sub_preset_maker_ = "/" + sub_preset_maker_;
//         // }
//         // sub_preset_maker_ = namespace_ + sub_preset_maker_;
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_maker_, SUB_QUEUE_SIZE, &Runner::callbackPresetMaker, this, INTEGRATED);

//       sub_preset_remover_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "delete_preset"});
//         // if(sub_preset_remover_.front() != '/'){
//         //   sub_preset_remover_ = "/" + sub_preset_remover_;
//         // }
//         // sub_preset_remover_ = namespace_ + sub_preset_remover_;
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::PresetCmd>(sub_preset_remover_, SUB_QUEUE_SIZE, &Runner::callbackPresetRemover, this, INTEGRATED);

//       sub_safety_checker_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "safety_check"});
//       ROS2::Node::getInstance().registerSubscriber<std_msgs::msg::Bool>(sub_safety_checker_, SUB_QUEUE_SIZE, &Runner::callbackSafetyChecker, this, INTEGRATED);

//       sub_set_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "create_protection_zone"});
//         // if(sub_set_protection_zone_.front() != '/'){
//         //   sub_set_protection_zone_ = "/" + sub_set_protection_zone_;
//         // }
//         // sub_set_protection_zone_ = namespace_ + sub_set_protection_zone_;
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_set_protection_zone_, SUB_QUEUE_SIZE, &Runner::callbackSetProtectionZone, this, INTEGRATED);

//       sub_delete_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "delete_protection_zone"});
//         // if(sub_delete_protection_zone_.front() != '/'){
//         //   sub_delete_protection_zone_ = "/" + sub_delete_protection_zone_;
//         // }
//         // sub_delete_protection_zone_ = namespace_ + sub_delete_protection_zone_;
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_delete_protection_zone_, SUB_QUEUE_SIZE, &Runner::callbackDeleteProtectionZone, this, INTEGRATED);

//       sub_update_protection_zone_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "update_protection_zone"});
//         // if(sub_update_protection_zone_.front() != '/'){
//         //   sub_update_protection_zone_ = "/" + sub_update_protection_zone_;
//         // }
//         // sub_update_protection_zone_ = namespace_ + sub_update_protection_zone_;
//       ROS2::Node::getInstance().registerSubscriber<add_sugv_msgs::msg::ProtectionZoneCmd>(sub_update_protection_zone_, SUB_QUEUE_SIZE, &Runner::callbackUpdateProtectionZone, this, INTEGRATED);




//       sub_state_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "state"});
//       ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::State>(sub_state_, SUB_QUEUE_SIZE, &Runner::callbackState, this, INTEGRATED);

//       sub_status_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "status"});
//       ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::Status>(sub_status_, SUB_QUEUE_SIZE, &Runner::callbackStatus, this, INTEGRATED);

//       sub_preset_list_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "action_list"});
//       ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ActionList>(sub_preset_list_, SUB_QUEUE_SIZE, &Runner::callbackPresetList, this, INTEGRATED);

//       sub_protection_zone_list_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "protection_zone_list"});
//       ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ProtectionZoneList>(sub_protection_zone_list_, SUB_QUEUE_SIZE, &Runner::callbackProtectionZoneList, this, INTEGRATED);

//       sub_preset_info_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "preset_info"});
//       ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ActionInfoArray>(sub_preset_info_, SUB_QUEUE_SIZE, &Runner::callbackPresetInfo, this, INTEGRATED);

//       sub_protection_zone_info_ = runnable_->getParameterManager()->get<std::string>({"system", "subscribe", "protection_zone_info"});
//       ROS2::Node::getInstance().registerSubscriber<kinova_msgs::msg::ProtectionZoneInfoArray>(sub_protection_zone_info_, SUB_QUEUE_SIZE, &Runner::callbackProtectionZoneInfo, this, INTEGRATED);

//       /*  
//           // clang-format on
//       */
//     }

//     void Runner::start() {
//       /**
//        * get message from topic
//        * if message is not received, it returns std::nullopt
//        */
//       // auto sub_msg =
//       //   ROS2::Node::getInstance().getSubscribeMessage<std_msgs::msg::Int64>(sub2_);
//       // if (sub_msg) {
//       //   runnable_->setC(convert<std_msgs::msg::Int64, int64_t>(*sub_msg));
//       // }
//     }

//     void Runner::end() {
//       /**
//        * publish message to topic
//        */
//       // ROS2::Node::getInstance().publishMessage<std_msgs::msg::Int64>(
//       //   pub2_, convert<int64_t, std_msgs::msg::Int64>(runnable_->getC()));
//     }

//     void Runner::diagnose() { logger_->info("ROS Runner diagnose"); }

//     void Runner::callbackEmergencyStop(const add_sugv_msgs::msg::EmergencyStopCmd &msg) {
//       e_stop_active_ = true;

//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::EmergencyStopCmd, std_msgs::msg::Bool>(msg);

//       ROS2::Node::getInstance().publishMessage<std_msgs::msg::Bool>(pub_emergency_stop_, converted_msg);
//       logger_->info("Published converted EmergencyStopCmd to Empty");
//     }

//     void Runner::callbackClearFault(const add_sugv_msgs::msg::ClearFaultCmd &msg) {
//       e_stop_active_ = false;

//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::ClearFaultCmd, std_msgs::msg::Bool>(msg);

//       ROS2::Node::getInstance().publishMessage<std_msgs::msg::Bool>(pub_clear_fault_, converted_msg);
//       logger_->info("Published converted ClearFaultCmd to Empty");
//     }

//     void Runner::callbackRemoteCmd(const add_sugv_msgs::msg::TwistCmd &msg) {
//       if (e_stop_active_) { 
//         logger_->warn("E-stop is active. Ignoring RemoteCmd.");
//         return;
//       }

//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::TwistCmd, geometry_msgs::msg::Twist>(msg);

//       ROS2::Node::getInstance().publishMessage<geometry_msgs::msg::Twist>(pub_remote_cmd_, converted_msg);
//       logger_->info("Published converted RemoteCmd to Twist");
//     }

//     void Runner::callbackGripperCmd(const add_sugv_msgs::msg::GripperCmd &msg) {
//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::GripperCmd, control_msgs::msg::GripperCommand>(msg);

//       ROS2::Node::getInstance().publishMessage<control_msgs::msg::GripperCommand>(pub_gripper_cmd_, converted_msg);
//       logger_->info("Published converted GripperCmd to GripperCommand");
//     }

//     void Runner::callbackJointCmd(const add_sugv_msgs::msg::JointCmd &msg) {
//       if (e_stop_active_) { 
//         logger_->warn("E-stop is active. Ignoring RemoteJointCmd");
//         return;
//       }

//       try {
//         auto converted_msg = Interface::convert<add_sugv_msgs::msg::JointCmd, trajectory_msgs::msg::JointTrajectory>(msg);

//         ROS2::Node::getInstance().publishMessage<trajectory_msgs::msg::JointTrajectory>(pub_joint_cmd_, converted_msg);
//       }
//       catch (const std::bad_any_cast& e) {
//         // bad_any_cast occured
//         logger_->error("Failed to convert RemoteJointCmd to JointTrajectory: bad_any_cast - {}", e.what());
//       } 
//       catch (const std::exception& e) {
//         // Other Exceptions....
//         logger_->error("Failed to convert RemoteJointCmd to JointTrajectory: {}", e.what());
//       }
//     }

//     void Runner::callbackState(const kinova_msgs::msg::State &msg) {
//       last_state_ = msg;
//       state_received_ = true;

//       callbackManipulatorStatus(); 
//     }

//     void Runner::callbackStatus(const kinova_msgs::msg::Status &msg) {
//       last_status_ = msg;
//       status_received_ = true;

//       callbackManipulatorStatus(); 
//     }

//     void Runner::callbackPresetList(const kinova_msgs::msg::ActionList &msg){
//       last_preset_list_ = msg.action_list;
//       preset_list_received_ = true;

//       callbackManipulatorStatus();
//     }

//     void Runner::callbackProtectionZoneList(const kinova_msgs::msg::ProtectionZoneList &msg){
//       last_protection_zone_list_ = msg.protection_zone_list;
//       protection_zone_list_received_ = true;

//       callbackManipulatorStatus();
//     }

//     void Runner::callbackManipulatorStatus() {
//       if (state_received_ && status_received_ && preset_list_received_ && protection_zone_list_received_) {
//         acelab::ManipulatorStatus manipulator_status_struct;
//         manipulator_status_struct.state = last_state_;    
//         manipulator_status_struct.status = last_status_;
//         manipulator_status_struct.action_list = last_preset_list_;
//         manipulator_status_struct.protection_zone_list = last_protection_zone_list_;

//         add_sugv_msgs::msg::ManipulatorStatus manipulator_status_msg = 
//             Interface::convert<acelab::ManipulatorStatus, add_sugv_msgs::msg::ManipulatorStatus>(manipulator_status_struct);

//         ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::ManipulatorStatus>(pub_manipulator_status_, manipulator_status_msg);

//         state_received_ = false;
//         status_received_ = false;
//         preset_list_received_ = false;
//         protection_zone_list_received_ = false;
//       }
//     }

//     void Runner::callbackPresetTargetCmd(const add_sugv_msgs::msg::PresetCmd &msg){
//       if (e_stop_active_) { 
//         logger_->warn("E-stop is active. Ignoring Cmd.");
//         return;
//       }

//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::PresetCmd, std_msgs::msg::String>(msg);

//       ROS2::Node::getInstance().publishMessage<std_msgs::msg::String>(pub_preset_target_cmd_, converted_msg);
//     }

//     void Runner::callbackPresetMaker(const add_sugv_msgs::msg::PresetCmd &msg){
//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::PresetCmd, kinova_msgs::msg::CreateAction>(msg);
//       ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::CreateAction>(pub_preset_maker_, converted_msg);
//     }

//     void Runner::callbackPresetRemover(const add_sugv_msgs::msg::PresetCmd &msg){
//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::PresetCmd, std_msgs::msg::String>(msg);

//       ROS2::Node::getInstance().publishMessage<std_msgs::msg::String>(pub_preset_remover_, converted_msg);
//     }

//     void Runner::callbackSafetyChecker(const std_msgs::msg::Bool &msg){
//       auto converted_msg = Interface::convert<std_msgs::msg::Bool, add_sugv_msgs::msg::SafetyCheck>(msg);

//       ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::SafetyCheck>(pub_safety_checker_,converted_msg);
//     }

//     void Runner::callbackPresetInfo(const kinova_msgs::msg::ActionInfoArray &msg){
//       auto converted_msg = Interface::convert<kinova_msgs::msg::ActionInfoArray, add_sugv_msgs::msg::PresetInfoArray>(msg);

//       ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::PresetInfoArray>(pub_preset_info_, converted_msg);
//     }

//     void Runner::callbackProtectionZoneInfo(const kinova_msgs::msg::ProtectionZoneInfoArray &msg){
//       auto converted_msg = Interface::convert<kinova_msgs::msg::ProtectionZoneInfoArray, add_sugv_msgs::msg::ProtectionZoneInfoArray>(msg);

//       ROS2::Node::getInstance().publishMessage<add_sugv_msgs::msg::ProtectionZoneInfoArray>(pub_protection_zone_info_, converted_msg);
//     }

//     void Runner::callbackSetProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg) {
//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::ProtectionZoneCmd, kinova_msgs::msg::ProtectionZone>(msg);

//       ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::ProtectionZone>(pub_set_protection_zone_, converted_msg);
//     }

//     void Runner::callbackDeleteProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg){
//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::ProtectionZoneCmd, kinova_msgs::msg::ProtectionZone>(msg);

//       ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::ProtectionZone>(pub_delete_protection_zone_, converted_msg);
//     }

//     void Runner::callbackUpdateProtectionZone(const add_sugv_msgs::msg::ProtectionZoneCmd &msg){
//       auto converted_msg = Interface::convert<add_sugv_msgs::msg::ProtectionZoneCmd, kinova_msgs::msg::ProtectionZone>(msg);

//       ROS2::Node::getInstance().publishMessage<kinova_msgs::msg::ProtectionZone>(pub_update_protection_zone_, converted_msg);
//     }

//   }  // namespace ROS
// }  // namespace acelab

