/**
 * @file interface.cc
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-15
 *
 * @copyright Copyright (c)ACELAB 2022
 *
 */

#include "my_project/runner/ros2/interface.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp" 

#include "control_msgs/msg/gripper_command.hpp"
#include "add_sugv_msgs/msg/emergency_stop_cmd.hpp"
#include "add_sugv_msgs/msg/clear_fault_cmd.hpp" 
#include "add_sugv_msgs/msg/twist_cmd.hpp"
#include "add_sugv_msgs/msg/gripper_cmd.hpp"
#include "add_sugv_msgs/msg/manipulator_status.hpp"
#include "add_sugv_msgs/msg/joint_cmd.hpp" 
#include "add_sugv_msgs/msg/preset_cmd.hpp" 
#include "add_sugv_msgs/msg/protection_zone_cmd.hpp"
#include "add_sugv_msgs/msg/safety_check.hpp"
#include "add_sugv_msgs/msg/preset_info.hpp"
#include "add_sugv_msgs/msg/preset_info_array.hpp"
#include "add_sugv_msgs/msg/protection_zone_info.hpp"
#include "add_sugv_msgs/msg/protection_zone_info_array.hpp"

#include "kinova_msgs/msg/state.hpp"
#include "kinova_msgs/msg/status.hpp"                
#include "kinova_msgs/msg/create_action.hpp"
#include "kinova_msgs/msg/protection_zone.hpp"
#include "kinova_msgs/msg/action_info.hpp"
#include "kinova_msgs/msg/action_info_array.hpp"
#include "kinova_msgs/msg/protection_zone_info.hpp"
#include "kinova_msgs/msg/protection_zone_info_array.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <iostream>

namespace acelab {
  geometry_msgs::msg::Twist twist_msg;

  // Emergency Stop Cmd
  template <>
  std_msgs::msg::Bool Interface::convert(const add_sugv_msgs::msg::EmergencyStopCmd &msg) {
    std_msgs::msg::Bool e_stop;
    e_stop.data = msg.emergency_stop;
    return e_stop;
  }

  // Clear Fault Cmd
  template <>
  std_msgs::msg::Bool Interface::convert(const add_sugv_msgs::msg::ClearFaultCmd &msg) {
    std_msgs::msg::Bool clear_fault;
    clear_fault.data = msg.clear_faults;
    return clear_fault;
  }

  // Twist Cmd
  template <>
  geometry_msgs::msg::Twist Interface::convert(const add_sugv_msgs::msg::TwistCmd &msg) {
    geometry_msgs::msg::Twist twist_msg;
    twist_msg.linear = msg.linear;
    twist_msg.angular = msg.angular;

    return twist_msg;
  }

  // Gripper Cmd
  template <>
  control_msgs::msg::GripperCommand Interface::convert(const add_sugv_msgs::msg::GripperCmd &msg) {
    control_msgs::msg::GripperCommand gripper_msg;
    gripper_msg.position = msg.position;
    gripper_msg.max_effort = msg.max_effort;

    return gripper_msg;
  }

  // GUI Joint Angle Delta Cmd
  template <>
  trajectory_msgs::msg::JointTrajectory Interface::convert(const add_sugv_msgs::msg::JointCmd &msg) {
    trajectory_msgs::msg::JointTrajectory joint_trajectory_msg;
    trajectory_msgs::msg::JointTrajectoryPoint point;

    // Move "Name"
    joint_trajectory_msg.joint_names = msg.name;

    point.positions = msg.position;
    point.velocities = msg.velocity;
    point.effort = msg.effort;

    // Set time stamp
    point.time_from_start.sec = 1;
    point.time_from_start.nanosec = 0;

    // Add Trajectory point
    joint_trajectory_msg.points.push_back(point);

    // Initialize Header
    joint_trajectory_msg.header.stamp.sec = 0;
    joint_trajectory_msg.header.stamp.nanosec = 0;
    
    return joint_trajectory_msg;
  }

  // Create the Preset
  template<>
  kinova_msgs::msg::CreateAction Interface::convert(const add_sugv_msgs::msg::PresetCmd &msg){
    kinova_msgs::msg::CreateAction action_maker;
    action_maker.name = msg.preset_name;

    // Joint angles handle
    action_maker.target_joint_angles.clear();
    for (size_t i = 0; i < 6; ++i) {
      if (i < msg.target_joint_angles.size()) {
          double angle_deg = msg.target_joint_angles[i];
          action_maker.target_joint_angles.push_back(angle_deg);
      } 
      else {
          action_maker.target_joint_angles.push_back(0.0);
      }
    }

    return action_maker;
  }

  // Delete and Play the Preset
  template<>
  std_msgs::msg::String Interface::convert(const add_sugv_msgs::msg::PresetCmd &msg){
    std_msgs::msg::String action_name;
    action_name.data = msg.preset_name;

    return action_name;
  }

  // Protection Zone Cmd 
  template <>
  kinova_msgs::msg::ProtectionZone Interface::convert(const add_sugv_msgs::msg::ProtectionZoneCmd &msg) {
    kinova_msgs::msg::ProtectionZone setting_info;

    if (msg.mode == 1) { // CREATE mode
        // Protection Zone Handle Setting
        setting_info.handle.identifier = msg.zone_id;
        setting_info.handle.permission = 0;

        // Protection Zone's name & status of activation
        setting_info.name = msg.zone_name;
        setting_info.is_enabled = msg.is_enabled;
        setting_info.mode = msg.mode;

        // Shape & position
        setting_info.shape.shape_type = msg.shape_type;

        // Assign position
        if (msg.position.size() >= 3) {
            setting_info.shape.origin.x = msg.position[0];
            setting_info.shape.origin.y = msg.position[1];
            setting_info.shape.origin.z = msg.position[2];
        } else {
            setting_info.shape.origin.x = 0.0f;
            setting_info.shape.origin.y = 0.0f;
            setting_info.shape.origin.z = 0.0f;
        }

        // Assign orientation (Only meaningful for certain shapes)
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        if (msg.orientation.size() >= 3) {
            roll = msg.orientation[0] * M_PI / 180.0;   // Convert degrees to radians
            pitch = msg.orientation[1] * M_PI / 180.0;
            yaw = msg.orientation[2] * M_PI / 180.0;
        }
        // Convert Euler angles to rotation matrix
        tf2::Quaternion quaternion;
        quaternion.setRPY(roll, pitch, yaw);
        quaternion.normalize();
        tf2::Matrix3x3 rotation_matrix(quaternion);

        // Set orientation matrix rows
        setting_info.shape.orientation.row1 = {
            static_cast<float>(rotation_matrix.getRow(0).getX()),
            static_cast<float>(rotation_matrix.getRow(0).getY()),
            static_cast<float>(rotation_matrix.getRow(0).getZ())
        };
        setting_info.shape.orientation.row2 = {
            static_cast<float>(rotation_matrix.getRow(1).getX()),
            static_cast<float>(rotation_matrix.getRow(1).getY()),
            static_cast<float>(rotation_matrix.getRow(1).getZ())
        };
        setting_info.shape.orientation.row3 = {
            static_cast<float>(rotation_matrix.getRow(2).getX()),
            static_cast<float>(rotation_matrix.getRow(2).getY()),
            static_cast<float>(rotation_matrix.getRow(2).getZ())
        };

        // Set dimensions based on shape_type
        setting_info.shape.dimensions.clear();
        switch (msg.shape_type) {
            case 1: // CYLINDER
                if (msg.dimensions.size() >= 2) {
                    // radius, height
                    setting_info.shape.dimensions.push_back(msg.dimensions[0]); // radius
                    setting_info.shape.dimensions.push_back(msg.dimensions[1]); // height
                } 
                else {
                    // Handle insufficient dimensions
                    // Set defaults or throw an error
                    float radius = (msg.dimensions.size() >= 1) ? msg.dimensions[0] : 0.0f;
                    setting_info.shape.dimensions.push_back(radius); // radius
                    setting_info.shape.dimensions.push_back(0.0f);   // height
                }
                break;
            case 2: // SPHERE
                if (msg.dimensions.size() >= 1) {
                    // radius
                    setting_info.shape.dimensions.push_back(msg.dimensions[0]); // radius
                } 
                else {
                    // Set default radius
                    setting_info.shape.dimensions.push_back(0.0f);
                }
                break;
            case 3: // RECTANGULAR_PRISM
                if (msg.dimensions.size() >= 3) {
                    // width(x), depth(y), height(z)
                    setting_info.shape.dimensions.push_back(msg.dimensions[0]); // width
                    setting_info.shape.dimensions.push_back(msg.dimensions[1]); // depth
                    setting_info.shape.dimensions.push_back(msg.dimensions[2]); // height
                } 
                else {
                    // Handle insufficient dimensions
                    // Set defaults or fill missing dimensions with 0.0
                    for (size_t i = 0; i < 3; ++i) {
                        float dim = (i < msg.dimensions.size()) ? msg.dimensions[i] : 0.0f;
                        setting_info.shape.dimensions.push_back(dim);
                    }
                }
                break;
            default:
                // Unsupported shape_type
                // Handle accordingly, perhaps log an error or set default dimensions
                setting_info.shape.dimensions.clear();
                break;
        }

        // Envelope Thickness Info
        setting_info.shape.envelope_thickness = msg.envelope_thickness;

        // Set envelope_limitations (Array of CartesianLimitation)
        setting_info.envelope_limitations.clear();
        kinova_msgs::msg::CartesianLimitation envelope_limitation;
        envelope_limitation.type = msg.shape_type;
        envelope_limitation.translation = msg.envelope_target_speed;
        envelope_limitation.orientation = 0.0f;
        setting_info.envelope_limitations.push_back(envelope_limitation);

        return setting_info;

    } 
    else if (msg.mode == 2) { // DELETE mode
        setting_info.name = msg.zone_name;
        setting_info.handle.identifier = 0;
        setting_info.handle.permission = 0;

        setting_info.is_enabled = false;
        setting_info.mode = msg.mode;

        setting_info.shape.shape_type = 0;

        setting_info.shape.origin.x = 0.0f;
        setting_info.shape.origin.y = 0.0f;
        setting_info.shape.origin.z = 0.0f;

        setting_info.shape.orientation.row1 = {1.0f, 0.0f, 0.0f};
        setting_info.shape.orientation.row2 = {0.0f, 1.0f, 0.0f};
        setting_info.shape.orientation.row3 = {0.0f, 0.0f, 1.0f};

        setting_info.shape.dimensions.clear();
        setting_info.shape.envelope_thickness = 0.0f;

        setting_info.limitations.clear();
        setting_info.envelope_limitations.clear();

        return setting_info;
      }
      else if (msg.mode == 3){  // Update(control) mode
        setting_info.name = msg.zone_name;
        setting_info.handle.identifier = 0;
        setting_info.handle.permission = 0;

        setting_info.is_enabled = msg.is_enabled;
        setting_info.mode = msg.mode;

        setting_info.shape.shape_type = 0;

        setting_info.shape.origin.x = 0.0f;
        setting_info.shape.origin.y = 0.0f;
        setting_info.shape.origin.z = 0.0f;

        setting_info.shape.orientation.row1 = {1.0f, 0.0f, 0.0f};
        setting_info.shape.orientation.row2 = {0.0f, 1.0f, 0.0f};
        setting_info.shape.orientation.row3 = {0.0f, 0.0f, 1.0f};

        setting_info.shape.dimensions.clear();
        setting_info.shape.envelope_thickness = 0.0f;

        setting_info.limitations.clear();
        setting_info.envelope_limitations.clear();

        return setting_info;
      }


      // Default return for unsupported modes
      return setting_info;
  }

  // Safety Check
  template<>
  add_sugv_msgs::msg::SafetyCheck Interface::convert(const std_msgs::msg::Bool &msg){
    add_sugv_msgs::msg::SafetyCheck wrapped_msg;
    wrapped_msg.is_touch= msg.data;

    return wrapped_msg;
  }

  // Manipulator Status
  template<>
  add_sugv_msgs::msg::ManipulatorStatus Interface::convert(const ManipulatorStatus& manipulator_status_struct) {
    add_sugv_msgs::msg::ManipulatorStatus manipulator_status_msg;

    // Copy State Data
    manipulator_status_msg.arm_state = static_cast<int16_t>(manipulator_status_struct.state.arm_state.state);   // Arm state
    manipulator_status_msg.joint_angles = manipulator_status_struct.state.joint_angles.angles;                  // Joint angles
    manipulator_status_msg.gripper_position = manipulator_status_struct.state.gripper_state.position;           // Gripper position

    manipulator_status_msg.preset_list = manipulator_status_struct.action_list;
    manipulator_status_msg.protection_zone_list = manipulator_status_struct.protection_zone_list;
    
    // Cartesian pose (Position and Orientation) from State
    manipulator_status_msg.end_effector_position.position.x = manipulator_status_struct.status.cartesian_pose.x;
    manipulator_status_msg.end_effector_position.position.y = manipulator_status_struct.status.cartesian_pose.y;
    manipulator_status_msg.end_effector_position.position.z = manipulator_status_struct.status.cartesian_pose.z;
    
    tf2::Quaternion quaternion;
    quaternion.setRPY(
        manipulator_status_struct.status.cartesian_pose.roll * M_PI / 180.0,
        manipulator_status_struct.status.cartesian_pose.pitch * M_PI / 180.0,
        manipulator_status_struct.status.cartesian_pose.yaw * M_PI / 180.0);
    quaternion.normalize(); 

    // Assign Quaternion to orientation
    manipulator_status_msg.end_effector_position.orientation.x = quaternion.x();
    manipulator_status_msg.end_effector_position.orientation.y = quaternion.y();
    manipulator_status_msg.end_effector_position.orientation.z = quaternion.z();
    manipulator_status_msg.end_effector_position.orientation.w = quaternion.w();

    // Copy Status Data
    manipulator_status_msg.operating_mode = static_cast<int8_t>(manipulator_status_struct.status.operating_mode.operating_mode);
    manipulator_status_msg.servoing_mode = static_cast<int8_t>(manipulator_status_struct.status.servoing_mode.servoing_mode);


    size_t num_actuators = manipulator_status_struct.status.actuator_feedback.size();

    for (size_t i = 0; i < num_actuators; i++) {
      auto& actuator_feedback = manipulator_status_struct.status.actuator_feedback[i];

      // Assuming that actuator feedback velocity and torque are relevant for end_effector_force and velocity
      manipulator_status_msg.velocity.push_back(actuator_feedback.velocity[i]);
      manipulator_status_msg.end_effector_force.push_back(actuator_feedback.torque[i]);
    }

    return manipulator_status_msg;
  }

  // Preset Info Array
  template<>
  add_sugv_msgs::msg::PresetInfoArray Interface::convert(const kinova_msgs::msg::ActionInfoArray &msg) {
    add_sugv_msgs::msg::PresetInfoArray info_Array_msg;

    // Use msg.info instead of msg.presets
    for (const auto& action_info : msg.info) {
      add_sugv_msgs::msg::PresetInfo info_msg;

      info_msg.preset_name = action_info.name;        // Use preset_name instead of name
      info_msg.pose = action_info.pose;
      info_msg.joint_angles = action_info.joint_angles;

      info_Array_msg.presets.push_back(info_msg);  
    }

    return info_Array_msg;
  }
  
  //Protection Zone Info Array
  template<>
  add_sugv_msgs::msg::ProtectionZoneInfoArray Interface::convert(const kinova_msgs::msg::ProtectionZoneInfoArray &msg) {
    add_sugv_msgs::msg::ProtectionZoneInfoArray converted_msg;

    for (const auto &info : msg.info) {
      add_sugv_msgs::msg::ProtectionZoneInfo converted_info;

      converted_info.zone_name = info.name;
      converted_info.origin = info.origin;
      converted_info.dimensions = info.dimensions;
      converted_info.shape_type = info.shape_type;
      converted_info.is_enabled = info.is_enabled;
      converted_info.envelope_thickness = info.envelope_thickness;
      converted_info.envelope_target_speed = info.envelope_target_speed;

      converted_msg.zones.push_back(converted_info);
    }

    return converted_msg;
  }

}  // namespace acelab
