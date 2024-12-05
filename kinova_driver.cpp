#include <cstdio>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <thread>
#include <Eigen/Geometry>
#include <atomic>
#include <map>
#include <set>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <cerrno>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionManager.h>
#include <RouterClient.h>
#include <TransportClientTcp.h>
#include "utilities.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <control_msgs/msg/gripper_command.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include "kinova_msgs/msg/protection_zone.hpp"
#include "kinova_msgs/msg/protection_zone_handle.hpp"
#include "kinova_msgs/msg/zone_shape.hpp"
#include "kinova_msgs/msg/point.hpp"
#include "kinova_msgs/msg/base_rotation_matrix.hpp"
#include "kinova_msgs/msg/cartesian_limitation.hpp"
#include "kinova_msgs/msg/create_action.hpp"
#include "kinova_msgs/msg/operating_mode.hpp"
#include "kinova_msgs/msg/servoing_mode.hpp"
#include "kinova_msgs/msg/pose_info.hpp"
#include "kinova_msgs/msg/actuator_feedback.hpp"
#include "kinova_msgs/msg/status.hpp"
#include "kinova_msgs/msg/current_pose.hpp"
#include "kinova_msgs/msg/joint_angles.hpp"
#include "kinova_msgs/msg/gripper_state.hpp"
#include "kinova_msgs/msg/arm_state.hpp"
#include "kinova_msgs/msg/state.hpp"
#include "kinova_msgs/msg/action_info.hpp"
#include "kinova_msgs/msg/action_info_array.hpp"
#include "kinova_msgs/msg/protection_zone_info.hpp"
#include "kinova_msgs/msg/protection_zone_info_array.hpp"
#include "kinova_msgs/msg/action_list.hpp"
#include "kinova_msgs/msg/protection_zone_list.hpp"

// URDF and KDL Headers
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>

// ament_index_cpp header
#include "ament_index_cpp/get_package_share_directory.hpp"

#define PORT 10000
#define ACTION_WAITING_TIME std::chrono::milliseconds(500)

// Suppress warnings about anonymous structs
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"

const float DEADZONE_THRESHOLD = 1e-5;

namespace k_api = Kinova::Api;

std::atomic<bool> collision_detected{false};

// Start of StatusNode Class
class StatusNode
{
public:
  StatusNode(std::shared_ptr<rclcpp::Node> node, k_api::Base::BaseClient *base, k_api::BaseCyclic::BaseCyclicClient *base_cyclic)
      : node_(node), base_(base), base_cyclic_(base_cyclic)
  {
    // State and Status Publishers
    state_publisher_ = node_->create_publisher<kinova_msgs::msg::State>("current_state", 10);                                          // Wrapped

    status_publisher_ = node_->create_publisher<kinova_msgs::msg::Status>("current_status", 10);                                       // Wrapped

    action_list_publisher_ = node_->create_publisher<kinova_msgs::msg::ActionList>("action_list", 10);                                 // Wrapped

    protection_zone_list_publisher_ = node_->create_publisher<kinova_msgs::msg::ProtectionZoneList>("protection_zone_list", 10);       // Wrapped

    action_info_publisher_ = node_->create_publisher<kinova_msgs::msg::ActionInfoArray>("action_info", 10);                            // Wrapped

    protection_zone_info_publisher_ = node_->create_publisher<kinova_msgs::msg::ProtectionZoneInfoArray>("protection_zone_info", 10);  // Wrapped

    safety_check_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("check_safety", 10);                                        // Wrapped

    // Load URDF File & Create KDL Tree
    std::string package_share_directory = ament_index_cpp::get_package_share_directory("kinova_driver");
    std::string urdf_file = package_share_directory + "/urdf/gen3.urdf";

    if (!load_robot_model(urdf_file))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load robot model from URDF file.");
      return;
    }

    // Thread for Collision check
    collision_thread_ = std::thread(&StatusNode::check_protection_zone_thread, this);

    // Timer for periodic publishing of state and status
    status_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&StatusNode::publish_status, this));
  }

  ~StatusNode() {
    if(collision_thread_.joinable()){
      run_collision_thread_ = false;
      collision_thread_.join();
    }
  }

private:
  const float FINE_TUNE_TOLERANCE = 0.02;

  /**
   * @brief Independent Thread for "Safety Check" -> Functional Safety Function to protect the robot that collides with protection zones.
   * 1. Load URDF and create KDL Chain
   * 2. Run loop to check the collision between protection zone and robot
   * 3. get result of collision by using check_protection_zone() func.
   * 4. If Collision occured, stop the robot and publish topic includes the collision result(bool)
   * 
   * Check Collision, and publish the result 
   */
  void check_protection_zone_thread(){
    while(run_collision_thread_){
      
      // auto loop_start_time = std::chrono::steady_clock::now();

      bool collision = check_protection_zone();
      collision_detected.store(collision);

      std_msgs::msg::Bool is_touch;
      is_touch.data = collision;
      safety_check_publisher_->publish(is_touch);

      // auto loop_end_time = std::chrono::steady_clock::now();

      // auto cost = std::chrono::duration_cast<std::chrono::milliseconds>(loop_end_time - loop_start_time);
      // RCLCPP_INFO(node_->get_logger(), "Loop duration: %ld milliseconds", cost);

      std::this_thread::sleep_for(std::chrono::milliseconds(1));  // 1000hz
    }
  }

  // Load Robot Model for KDL(Kinematics & Dynamics Library) Solver
  bool load_robot_model(const std::string &urdf_file){
    urdf::Model urdf_model;
    if (!urdf_model.initFile(urdf_file))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to parse URDF file: %s", urdf_file.c_str());
      return false;
    }

    // When Failed to construct the KDL Tree based on URDF FIle. -> Check urdf file in "{current_pkg_folder}/third_party/URDF/gen3.urdf"
    if (!kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to construct KDL tree");
      return false;
    }

    // Create KDL Chain from Base Link to End Effector Link
    // KDL Chain = Robot's specific Joints or Arms. Used as lower structure of KDL Tree
    // KDL Tree = Robot's Whole Structure

    std::string base_link = "left_base_link";                   // Base Link name from URDF File
    std::string end_effector_link = "left_end_effector_link";   // End Effector Link name from URDF File

    if (!kdl_tree_.getChain(base_link, end_effector_link, kdl_chain_))
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get KDL chain from %s to %s", base_link.c_str(), end_effector_link.c_str());
      return false;
    }

    // Create Forward Kinematics Solver
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

    return true;    
  }

      /**
       * @brief Set the zone dimensions object
       * 
       * Read Robot's current enabled Protection Zones's Positions & Size
       * 
       * @param zone = Object of Protection zone
       * @param dim_x = Dimension for X axis
       * @param dim_y = Dimension for Y axis
       * @param dim_z = Dimension for Z axis
       * @param envelope_thickness = Thickness of outer shield
       */
      void set_zone_dimensions(const k_api::Base::ProtectionZone &zone, float &dim_x, float &dim_y, float &dim_z, float &envelope_thickness)
      {
        auto shape_type = zone.shape().shape_type();
        envelope_thickness = zone.shape().envelope_thickness();

        // Protection Zone's Shape = Rectangular Prism 
        if (shape_type == k_api::Base::ShapeType::RECTANGULAR_PRISM && zone.shape().dimensions_size() >= 3)
        {
          dim_x = zone.shape().dimensions(0);
          dim_y = zone.shape().dimensions(1);
          dim_z = zone.shape().dimensions(2);
        }

        // Protection Zone's Shape = Cylinder
        else if (shape_type == k_api::Base::ShapeType::CYLINDER && zone.shape().dimensions_size() >= 2)
        {
          dim_x = zone.shape().dimensions(0);
          dim_y = zone.shape().dimensions(1);
        }

        // Protection Zone's Shape = Sphere
        else if (shape_type == k_api::Base::ShapeType::SPHERE && zone.shape().dimensions_size() >= 1)
        {
          dim_x = zone.shape().dimensions(0);
        }
        
        dim_x += 2 * envelope_thickness;
        dim_y += 2 * envelope_thickness;
        dim_z += 2 * envelope_thickness;
      }

  /**
   * @brief Inspect the robot's each joints for collisions with the protection Zone
   * 1. Get Robot's current joint angles
   * 2. Compute each joint's Pose by using KDL Tree
   * 3. Get robot's all protection zones and calculate for collision
   * 4. Return the value of collision check
   * 
   * @return true = Robot is in the protection Zone
   * @return false = Robot is not in the protection Zone
   */
  bool check_protection_zone()
  {
    // Get Current Joint Angles
    int num_joints = 0;
    k_api::Base::JointAngles current_joint_angles;
    try
    {
      current_joint_angles = base_->GetMeasuredJointAngles();
      num_joints = current_joint_angles.joint_angles_size();
      /* code */
    }
    catch(const std::exception& e)
    {
      std::cerr << e.what() << '\n';
      return false;
    }
    

    // Verify if the number of joints in the robot matches the number of joints in the KDL chain
    if (num_joints != kdl_chain_.getNrOfJoints())
    {
      RCLCPP_ERROR(node_->get_logger(), "Joint count mismatch between robot and URDF model");
      return false;
    }

    // Create a KDL joint array to store the current joint positions in radians
    // KDL JntArray = joint, data structure for representing data
    KDL::JntArray kdl_joint_positions(kdl_chain_.getNrOfJoints());

    // Populate the KDL joint array with the current joint angles converted to radians
    for (int i = 0; i < num_joints; i++)
    {
      // Change Joint's angle degree to Radian
      double joint_value = current_joint_angles.joint_angles(i).value() * M_PI / 180.0;   // Change degree to Radian
      kdl_joint_positions(i) = joint_value;
    }

    // Initialize a vector to store the pose (KDL::Frame) of each joint
    std::vector<KDL::Frame> joint_frames;
    joint_frames.resize(kdl_chain_.getNrOfSegments());    // Resize to the number of segments in the KDL chain

    KDL::Frame current_frame = KDL::Frame::Identity();    // Start with an identity frame

    unsigned int joint_idx = 0;

    // Iterate through each segment in the KDL chain to calculate the pose of each joint
    for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
    {
      const KDL::Segment &segment = kdl_chain_.getSegment(i);  // KDL Segment = Combination of single links and joints that make up the chain
      const KDL::Joint &joint = segment.getJoint();

      if (joint.getType() != KDL::Joint::None)
      {
        double joint_position = kdl_joint_positions(joint_idx);
        current_frame = current_frame * segment.pose(joint_position);   // Compute the new pose
        joint_idx++; 
      }
      else
      {
        current_frame = current_frame * segment.pose(0.0);    // Fixed joint, use 0.0 as the joint position
      }

      joint_frames[i] = current_frame;
    }

    // Get all Protection Zones
    const auto protection_zones = base_->ReadAllProtectionZones().protection_zones();

    // Check Collision for each Protection Zones
    for (const auto &zone : protection_zones)
    {
      if (!zone.is_enabled())
      {
        continue; // If protection zone is disabled, ignore that zone and continue
      }

      if (zone.name() == "Base Protection Zone")
      {
          continue; // "Base" Protection zone is exception of collision Check.
      }

      // Get Zone's position and dimensions
      auto origin = zone.shape().origin();
      float dim_x = 0, dim_y = 0, dim_z = 0, envelope_thickness = 0;
      set_zone_dimensions(zone, dim_x, dim_y, dim_z, envelope_thickness);

      // Retrieve the orientation of the protection zone and construct a rotation matrix
      auto orientation = zone.shape().orientation();
      Eigen::Matrix3d rotation_matrix;
      rotation_matrix << orientation.row1().column1(), orientation.row1().column2(), orientation.row1().column3(),
          orientation.row2().column1(), orientation.row2().column2(), orientation.row2().column3(),
          orientation.row3().column1(), orientation.row3().column2(), orientation.row3().column3();

      // Create an affine transformation for the protection zone
      Eigen::Affine3d zone_transform = Eigen::Affine3d::Identity();
      zone_transform.translation() << origin.x(), origin.y(), origin.z();
      zone_transform.linear() = rotation_matrix;

      // Check Collision with each link (between joints)
      for (unsigned int i = 2; i < joint_frames.size(); ++i)
      {
        // Get the start and end positions of the link in world coordinates
        Eigen::Vector3d start_pos(joint_frames[i - 1].p.x(), joint_frames[i - 1].p.y(), joint_frames[i - 1].p.z());
        Eigen::Vector3d end_pos(joint_frames[i].p.x(), joint_frames[i].p.y(), joint_frames[i].p.z());

        // Transform the start and end positions into the protection zone's coordinate system
        Eigen::Vector3d start_in_zone = zone_transform.inverse() * start_pos;
        Eigen::Vector3d end_in_zone = zone_transform.inverse() * end_pos;

        bool collision = false;

        // Check collision based on the type of protection zone (rectangular prism, cylinder, or sphere)
        if (zone.shape().shape_type() == k_api::Base::ShapeType::RECTANGULAR_PRISM)
        {
          collision = line_segment_intersects_rectangular_prism(start_in_zone, end_in_zone, dim_x, dim_y, dim_z);
        }
        else if (zone.shape().shape_type() == k_api::Base::ShapeType::CYLINDER)
        {
          collision = line_segment_intersects_cylinder(start_in_zone, end_in_zone, dim_x / 2, dim_y);
        }
        else if (zone.shape().shape_type() == k_api::Base::ShapeType::SPHERE)
        {
          collision = line_segment_intersects_sphere(start_in_zone, end_in_zone, dim_x / 2);
        }

        // If a collision is detected, log the warning and stop the robot
        if (collision)
        {
          RCLCPP_WARN(node_->get_logger(), "Collision Detected with Protection Zone '%s' at link %d-%d", zone.name().c_str(), i - 1, i);
          try{
            base_->Stop();
          }
          catch(const k_api::KDetailedException &ex)
          {
            RCLCPP_ERROR(node_->get_logger(), "Failed to stop the robot: %s", ex.what());
          }
          
          return true;   // Return true to indicate a collision occurred
        }
      }
    }

    return false;        // Return false if no collisions were detected
  }

  /**
   * @brief Function to check if a line segment intersects a RECTANGULAR_PRISM
   * 
   * @param start = Starting point of a line Segment
   * @param end = End point of a line Segment
   * @param width = The Width of a rectangular parallelepiped
   * @param depth = The Depth of a rectangular parallelepiped
   * @param height = The Height of a rectangular parallelepiped
   * @return true  = The Link is in the Zone
   * @return false = The Link is not in the Zone
   */
  bool line_segment_intersects_rectangular_prism(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                                 double width, double depth, double height)
  {
    // Axis-Aligned Bounding Box (AABB) of the rectangular prism centered at origin
    Eigen::Vector3d box_min(-width / 2 - FINE_TUNE_TOLERANCE, -depth / 2 - FINE_TUNE_TOLERANCE, -height / 2 - FINE_TUNE_TOLERANCE);
    Eigen::Vector3d box_max(width / 2 + FINE_TUNE_TOLERANCE, depth / 2 + FINE_TUNE_TOLERANCE, height / 2 + FINE_TUNE_TOLERANCE);

    return line_segment_intersects_aabb(start, end, box_min, box_max);
  }

  /**
   * @brief Function to check if a line segment intersects a CYLINDER aligned along the z-axis
   * 
   * @param start = Starting point of a line Segment
   * @param end = End point of a line Segment
   * @param radius = Radius of Cylinder
   * @param height = Height of Cylinder
   * @return true = The link is in the Zone.
   * @return false = The Link is not in the Zone.
   */
  bool line_segment_intersects_cylinder(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                        double radius, double height)
  {
    // Cylinder is centered at origin, extends from -height/2 to height/2 along z-axis
    // Can perform a simplified collision check

    // First, check if the segment intersects the cylindrical volume
    bool intersects_cylinder = line_segment_intersects_cylinder_along_z(start, end, radius, height);

    return intersects_cylinder;
  }

  /**
   * @brief Function to check if a line segment intersects a SPHERE centered at origin
   * 
   * @param start = Starting point of a line Segment
   * @param end = End point of a line Segment
   * @param radius = Radius of Sphere
   * @return true  = The Link is in the Zone.
   * @return false = The Link is not in the Zone.
   */
  bool line_segment_intersects_sphere(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                      double radius)
  {
    // Sphere is centered at origin
    double radius_with_tolerance = radius + FINE_TUNE_TOLERANCE;

    // Compute the shortest distance from the sphere center to the line segment
    double distance = point_line_distance(Eigen::Vector3d::Zero(), start, end);

    return (distance <= radius_with_tolerance);
  }

  /**
   * @brief Helper function to compute the shortest distance from a point to a line segment 
   * 
   * @param point = A Reference Point
   * @param start = Starting point of a line Segment
   * @param end = End point of a line Segment
   * @return double = The Shortest distance
   */
  double point_line_distance(const Eigen::Vector3d& point, const Eigen::Vector3d& start, const Eigen::Vector3d& end)
  {
    Eigen::Vector3d line = end - start;
    Eigen::Vector3d line_to_point = point - start;

    double t = line.dot(line_to_point) / line.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));

    Eigen::Vector3d projection = start + t * line;
    return (point - projection).norm();
  }

  /**
   * @brief Examine if the line segment intersects the axis-aligned bounding box(AABB)
   *  AABB = Axis-Aligned Bounding Box
   * 
   * @param start = Starting point of line Segment
   * @param end = End point of line Segment
   * @param box_min = Minimum coordinates of the box
   * @param box_max = Maximum coordinates of the box
   * @return true = Link is in the Zone
   * @return false = Link is in the Zone
   */
  bool line_segment_intersects_aabb(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                    const Eigen::Vector3d& box_min, const Eigen::Vector3d& box_max)
  {
    // Liang-Barsky algorithm for line-AABB intersection
    double tmin = 0.0;
    double tmax = 1.0;
    Eigen::Vector3d d = end - start;

    for (int i = 0; i < 3; ++i)
    {
      if (std::abs(d[i]) < 1e-8)
      {
        if (start[i] < box_min[i] || start[i] > box_max[i])
          return false;
      }
      else
      {
        double ood = 1.0 / d[i];
        double t1 = (box_min[i] - start[i]) * ood;
        double t2 = (box_max[i] - start[i]) * ood;

        if (t1 > t2)
          std::swap(t1, t2);

        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);

        if (tmin > tmax)
          return false;
      }
    }
    return true;
  }

  /**
   * @brief Function to check if a line segment intersects a cylinder aligned along the z-axis 
   * 
   * @param start = Stating point of line Segment
   * @param end = End point of line Segment
   * @param radius = Radius of a Cylinder
   * @param height = Height of a Cylinder
   * @return true = Link is in the Zone
   * @return false = Link is not in the Zone
   */
  bool line_segment_intersects_cylinder_along_z(const Eigen::Vector3d& start, const Eigen::Vector3d& end,
                                                double radius, double height)
  {
    double radius_with_tolerance = radius + FINE_TUNE_TOLERANCE;
    double half_height = height / 2 + FINE_TUNE_TOLERANCE;

    // Project the line segment onto the XY plane
    Eigen::Vector2d start_xy(start.x(), start.y());
    Eigen::Vector2d end_xy(end.x(), end.y());

    // Compute the shortest distance from the cylinder axis to the line segment projected onto XY plane
    double distance = point_line_distance_2d(Eigen::Vector2d::Zero(), start_xy, end_xy);

    // Check if the line segment is within the cylinder radius
    if (distance > radius_with_tolerance)
      return false;

    // Check if the line segment intersects the height of the cylinder
    double z_min = std::min(start.z(), end.z());
    double z_max = std::max(start.z(), end.z());

    if (z_min > half_height || z_max < -half_height)
      return false;

    return true;
  }

  /**
   * @brief Helper function to compute the shortest distance from a point to a line segment in 2D 
   * 
   * @param point = Represents the coordinates of the point in 2D space for which the shortest distance to the line segment will be computed
   * @param start  = Starting point of the line Segment
   * @param end = End point of the line Segment
   * @return double = The Shortest Euclidean distance from the point to the line segment defined by start and end.
   */
  double point_line_distance_2d(const Eigen::Vector2d& point, const Eigen::Vector2d& start, const Eigen::Vector2d& end)
  {
    Eigen::Vector2d line = end - start;
    Eigen::Vector2d line_to_point = point - start;

    double t = line.dot(line_to_point) / line.squaredNorm();
    t = std::max(0.0, std::min(1.0, t));

    Eigen::Vector2d projection = start + t * line;
    return (point - projection).norm();
  }


  void publish_status()
  {
    // auto start_time = std::chrono::steady_clock::now();

    // Call Kinova API async methods
    auto future_operating_mode = base_->GetOperatingMode_async();
    auto future_servoing_mode = base_->GetServoingMode_async();
    auto future_feedback = base_cyclic_->RefreshFeedback_async();
    auto future_joint_angles = base_->GetMeasuredJointAngles_async();
    auto future_arm_state = base_->GetArmState_async();
    auto future_trajectory_error = base_->GetTrajectoryErrorReport_async();
    auto future_actuator_count = base_->GetActuatorCount_async();

    k_api::Base::GripperRequest gripper_request;
    gripper_request.set_mode(k_api::Base::GRIPPER_POSITION);
    auto future_gripper_feedback = base_->GetMeasuredGripperMovement_async(gripper_request);

    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::UNSPECIFIED_ACTION);
    auto future_action_list = base_->ReadAllActions_async(action_type);

    auto future_protection_zone_list = base_->ReadAllProtectionZones_async();

    // Call Cache Updating func
    update_action_info_cache();
    // update_protection_zone_info_cache();

    // Cached Action Info Publish
    kinova_msgs::msg::ActionInfoArray action_info_table_msg;
    for (const auto &pair : action_info_cache_)
    {
        action_info_table_msg.info.push_back(pair.second);
    }
    action_info_publisher_->publish(action_info_table_msg);

    // // Cached Protection Zone Info Publish
    // kinova_msgs::msg::ProtectionZoneInfoArray protection_zone_info_table_msg;
    // for (const auto &pair : protection_zone_info_cache_)
    // {
    //     protection_zone_info_table_msg.info.push_back(pair.second);
    // }
    // protection_zone_info_publisher_->publish(protection_zone_info_table_msg);

    // Get Future results
    auto operating_mode_info = future_operating_mode.get();
    auto servoing_mode_info = future_servoing_mode.get();
    auto feedback = future_feedback.get();
    auto input_joint_angles = future_joint_angles.get();
    auto arm_state_info = future_arm_state.get();
    auto trajectory_error_report = future_trajectory_error.get();
    auto actuator_count = future_actuator_count.get();
    auto gripper_feedback = future_gripper_feedback.get();
    auto action_list = future_action_list.get();
    auto protection_zone_list = future_protection_zone_list.get();

    // --- Publish Status ---
    kinova_msgs::msg::Status status_msg;

    // Operating Mode & Servoing Mode 설정
    status_msg.operating_mode.operating_mode = operating_mode_info.operating_mode();
    status_msg.servoing_mode.servoing_mode = servoing_mode_info.servoing_mode();

    // Cartesian tool Pose Settings
    status_msg.cartesian_pose.x = feedback.base().tool_pose_x();
    status_msg.cartesian_pose.y = feedback.base().tool_pose_y();
    status_msg.cartesian_pose.z = feedback.base().tool_pose_z();
    status_msg.cartesian_pose.roll = feedback.base().tool_pose_theta_x();
    status_msg.cartesian_pose.pitch = feedback.base().tool_pose_theta_y();
    status_msg.cartesian_pose.yaw = feedback.base().tool_pose_theta_z();

    // Actuator Feedback Settings
    status_msg.actuator_feedback.resize(feedback.actuators_size());
    for (int i = 0; i < feedback.actuators_size(); i++)
    {
        auto actuator = feedback.actuators(i);
        kinova_msgs::msg::ActuatorFeedback actuator_feedback_msg;

        actuator_feedback_msg.position.push_back(actuator.position());
        actuator_feedback_msg.velocity.push_back(actuator.velocity());
        actuator_feedback_msg.torque.push_back(actuator.torque());
        // actuator_feedback_msg.current_motor.push_back(actuator.current_motor());
        // actuator_feedback_msg.voltage.push_back(actuator.voltage());
        // actuator_feedback_msg.temperature_motor.push_back(actuator.temperature_motor());
        // actuator_feedback_msg.temperature_core.push_back(actuator.temperature_core());
        // actuator_feedback_msg.status_flags.push_back(actuator.status_flags());
        // actuator_feedback_msg.jitter_comm.push_back(actuator.jitter_comm());
        // actuator_feedback_msg.fault_bank_a.push_back(actuator.fault_bank_a());
        // actuator_feedback_msg.fault_bank_b.push_back(actuator.fault_bank_b());
        // actuator_feedback_msg.warning_bank_a.push_back(actuator.warning_bank_a());
        // actuator_feedback_msg.warning_bank_b.push_back(actuator.warning_bank_b());

        status_msg.actuator_feedback[i] = actuator_feedback_msg;
    }

    // Publish Status
    status_publisher_->publish(status_msg);

    // --- Publish State ---
    kinova_msgs::msg::State state_msg;

    // Joint Angles Settings
    state_msg.joint_angles.angles.resize(input_joint_angles.joint_angles_size());
    for (int i = 0; i < input_joint_angles.joint_angles_size(); ++i)
    {
        state_msg.joint_angles.angles[i] = input_joint_angles.joint_angles(i).value();
    }

    // Arm State Settings
    state_msg.arm_state.state = arm_state_info.active_state();
    state_msg.arm_state.dof = actuator_count.count();

    if (trajectory_error_report.trajectory_error_elements_size() > 0)
    {
        auto current_error = trajectory_error_report.trajectory_error_elements(0);
        state_msg.arm_state.trajectory_error_report = current_error.error_type();
    }
    else
    {
        state_msg.arm_state.trajectory_error_report = 0;
    }

    // Gripper State Settings
    if (gripper_feedback.finger_size() > 0)
    {
        state_msg.gripper_state.position = gripper_feedback.finger(0).value();
    }
    else
    {
        state_msg.gripper_state.position = 0.0;
    }

    // Publish State
    state_publisher_->publish(state_msg);

    // --- Publish Action List ---
    std::vector<std::string> action_names;
    for (const auto &action : action_list.action_list())
    {
        action_names.push_back(action.name());
    }

    // Action List publish
    kinova_msgs::msg::ActionList action_list_msg;
    action_list_msg.action_list = action_names;
    action_list_publisher_->publish(action_list_msg);

    // --- Publish Protection Zone List ---
    std::vector<std::string> protection_zone_names;
    for (const auto &zone : protection_zone_list.protection_zones())
    {
        protection_zone_names.push_back(zone.name());
    }

    // Protection Zone List publish
    kinova_msgs::msg::ProtectionZoneList protection_zone_msg;
    protection_zone_msg.protection_zone_list = protection_zone_names;
    protection_zone_list_publisher_->publish(protection_zone_msg);

    // Publish Protection Zone Info
    kinova_msgs::msg::ProtectionZoneInfoArray protection_zone_info_array_msg;
    
    for(const auto &protection_zone : protection_zone_list.protection_zones()){
      kinova_msgs::msg::ProtectionZoneInfo protection_zone_info_msg;

      // Name
      protection_zone_info_msg.name = protection_zone.name();

      // Origin
      protection_zone_info_msg.origin = {protection_zone.shape().origin().x(),
                                         protection_zone.shape().origin().y(),
                                         protection_zone.shape().origin().z()};

      // Activated Status
      protection_zone_info_msg.is_enabled = protection_zone.is_enabled();

      // Dimension
      for (const auto &dim : protection_zone.shape().dimensions()){
        protection_zone_info_msg.dimensions.push_back(dim);
      }
      
      // Shape Type
      protection_zone_info_msg.shape_type = protection_zone.shape().shape_type();

      // Thickness
      protection_zone_info_msg.envelope_thickness = protection_zone.shape().envelope_thickness();

      // Envelope Target_speed
      if (protection_zone.envelope_limitations_size() > 0){
        auto envelope_limitation = protection_zone.envelope_limitations(0);
        protection_zone_info_msg.envelope_target_speed = envelope_limitation.translation();
      }
      else{
        // If Envelope limitations are NAN, set default value as 0.0
        protection_zone_info_msg.envelope_target_speed = 0.0;
      }

      // Push back to Array
      protection_zone_info_array_msg.info.push_back(protection_zone_info_msg);
    }

    protection_zone_info_publisher_->publish(protection_zone_info_array_msg);
    // // Check Loop time
    // auto end_time = std::chrono::steady_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    // RCLCPP_INFO(node_->get_logger(), "publish_status execution time: %ld ms", duration.count());
  }


  void update_action_info_cache()
  {
      // Set the Action Type to UNSPECIFIED_ACTION to retrieve all actions
      auto action_type = k_api::Base::RequestedActionType();
      action_type.set_action_type(k_api::Base::UNSPECIFIED_ACTION);

      // Retrieve all actions from the robot's API
      auto action_list = base_->ReadAllActions(action_type);

      // Create unordered set to save current action
      std::unordered_set<std::string> current_action_names;

      for (const auto &action : action_list.action_list())
      {
          // Save the names of all current actions
          current_action_names.insert(action.name());
      }

      // Find added or updated action
      for (const auto &action : action_list.action_list())
      {
          const std::string &action_name = action.name();

          // Check if the action is new (not in the cache)
          if (action_info_cache_.find(action_name) == action_info_cache_.end())
          {
              kinova_msgs::msg::ActionInfo action_info_msg;

              action_info_msg.name = action_name;

              // If the actions contains target joint angles, process them
              if (action.has_reach_joint_angles())
              {
                  const auto &target_joint_angles = action.reach_joint_angles().joint_angles();
                  for (int i = 0; i < target_joint_angles.joint_angles_size(); i++)
                  {
                      // Add joint angles to the message
                      action_info_msg.joint_angles.push_back(target_joint_angles.joint_angles(i).value());
                  }
                  try
                  {
                    // Get the target joint angles
                      auto joint_angles = action.reach_joint_angles().joint_angles();

                      // Compute FK
                      //* 1. KDL (x)
                      //* 2. std::async
                                        
                        // std::vector<std::future<kinova_msgs::msg::Pose>> futures;
                        // for (const auto &action : action_list.action_list())
                        // {
                            // if (action.has_reach_joint_angles())
                            // {
                                // const auto joint_angles = action.reach_joint_angles().joint_angles();
                                // futures.push_back(std::async(std::launch::async, [&base_](const auto &angles) {
                                    // return base_->ComputeForwardKinematics(angles);
                                // }, joint_angles));
                            // }
                        // }
                        // for (auto &future : futures)
                        // {
                            // try {
                                // auto pose = future.get();
                                // Pose 데이터를 처리
                            // } catch (...) {
                                // 예외 처리
                            // }
                        // }

                      // Compute Forward kinematics for the given joint angles
                      auto calculated_pose = base_->ComputeForwardKinematics(joint_angles, 0 ,{false, 0, 7000});

                      // Add the calculated pose to the message
                      action_info_msg.pose = {calculated_pose.x(), calculated_pose.y(), calculated_pose.z(),
                                              calculated_pose.theta_x(), calculated_pose.theta_y(), calculated_pose.theta_z()};
                  }
                  catch (const k_api::KDetailedException &ex)
                  {
                      std::cerr << "Unable to compute forward kinematics for action " << action_name << std::endl;
                  }
              }
              // Add the new action to the cache
              action_info_cache_[action_name] = action_info_msg;
          }
          else
          {
              /*
                T.B.D
              */
          }
      }

      // If action was deleted, delete from Cache
      for (const auto &prev_action_name : previous_action_names_)
      {
          if (current_action_names.find(prev_action_name) == current_action_names.end())
          {
              action_info_cache_.erase(prev_action_name);
          }
      }

      // Compare to next loop
      previous_action_names_ = current_action_names;
  }


  // void update_protection_zone_info_cache()
  // {
  //     auto protection_zone_list = base_->ReadAllProtectionZones();

  //     // Create Set to save current protection zone
  //     std::unordered_set<std::string> current_protection_zone_names;

  //     for (const auto &protection_zone : protection_zone_list.protection_zones())
  //     {
  //         current_protection_zone_names.insert(protection_zone.name());
  //     }

  //     // Find Added or updated protection Zone
  //     for (const auto &protection_zone : protection_zone_list.protection_zones())
  //     {
  //         const std::string &zone_name = protection_zone.name();

  //         // Check if the protection zone is new (not in cache)
  //         if (protection_zone_info_cache_.find(zone_name) == protection_zone_info_cache_.end())
  //         {
  //             kinova_msgs::msg::ProtectionZoneInfo protection_zone_info_msg;

  //             protection_zone_info_msg.name = zone_name;

  //             protection_zone_info_msg.origin = {protection_zone.shape().origin().x(),
  //                                               protection_zone.shape().origin().y(),
  //                                               protection_zone.shape().origin().z()};

  //             protection_zone_info_msg.is_enabled = protection_zone.is_enabled();

  //             // Save the dimensions of the protection zone
  //             for (const auto &dim : protection_zone.shape().dimensions())
  //             {
  //                 protection_zone_info_msg.dimensions.push_back(dim);
  //             }

  //             // Save the shape type of the protection zone
  //             protection_zone_info_msg.shape_type = protection_zone.shape().shape_type();

  //             // Sace the envelope thickness of the protection zone
  //             protection_zone_info_msg.envelope_thickness = protection_zone.shape().envelope_thickness();
              
  //             // If envelope limitations exist, save the target speed
  //             if(protection_zone.envelope_limitations_size() > 0){
  //               auto envelope_limitation = protection_zone.envelope_limitations(0);
  //               protection_zone_info_msg.envelope_target_speed = envelope_limitation.translation();
  //             }
  //             else{
  //               // Default to 0.0 if no envelope limitations exist
  //               protection_zone_info_msg.envelope_target_speed = 0.0;
  //             }

  //             // Add the new protection zone to the cache
  //             protection_zone_info_cache_[zone_name] = protection_zone_info_msg;
  //         }
  //         else
  //         {
  //             /*
  //               T.B.D
  //              */
  //         }
  //     }

  //     for (const auto &prev_zone_name : previous_protection_zone_names_)
  //     {
  //         if (current_protection_zone_names.find(prev_zone_name) == current_protection_zone_names.end())
  //         {
  //             protection_zone_info_cache_.erase(prev_zone_name);
  //         }
  //     }

  //     previous_protection_zone_names_ = current_protection_zone_names;
  // }

  rclcpp::Publisher<kinova_msgs::msg::State>::SharedPtr state_publisher_;
  rclcpp::Publisher<kinova_msgs::msg::Status>::SharedPtr status_publisher_;
  rclcpp::Publisher<kinova_msgs::msg::ActionList>::SharedPtr action_list_publisher_;
  rclcpp::Publisher<kinova_msgs::msg::ProtectionZoneList>::SharedPtr protection_zone_list_publisher_;
  rclcpp::Publisher<kinova_msgs::msg::ActionInfoArray>::SharedPtr action_info_publisher_;
  rclcpp::Publisher<kinova_msgs::msg::ProtectionZoneInfoArray>::SharedPtr protection_zone_info_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr safety_check_publisher_;
  rclcpp::TimerBase::SharedPtr status_timer_;

  KDL::Tree kdl_tree_;
  KDL::Chain kdl_chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  std::thread collision_thread_;
  std::atomic<bool> run_collision_thread_{true};

  std::shared_ptr<rclcpp::Node> node_;
  k_api::Base::BaseClient *base_;
  k_api::BaseCyclic::BaseCyclicClient *base_cyclic_;

  std::unordered_map<std::string, kinova_msgs::msg::ActionInfo> action_info_cache_;
  std::unordered_map<std::string, kinova_msgs::msg::ProtectionZoneInfo> protection_zone_info_cache_;

  std::unordered_set<std::string> previous_action_names_;
  std::unordered_set<std::string> previous_protection_zone_names_;
}; // End of StatusNode

// Start of ControlNode
class ControlNode
{
public:
  ControlNode(std::shared_ptr<rclcpp::Node> node, k_api::Base::BaseClient *base, 
              k_api::BaseCyclic::BaseCyclicClient *base_cyclic, k_api::SessionManager *session_manager,
              const k_api::Session::CreateSessionInfo &create_session_info)
      : base_(base), base_cyclic_(base_cyclic), gripper_stop_(false), estop_active_(false), session_manager_(session_manager), create_session_info_(create_session_info)
  {
    node_ = node;
    instance_ = this;

    joystick_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "twist_remote_cmd", 1, std::bind(&ControlNode::twist_callback, this, std::placeholders::_1));                 // Wrapped

    gui_end_effector_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "end_effector_pose", 10, std::bind(&ControlNode::EndEffector_callback, this, std::placeholders::_1));

    gui_joint_angle_subscriber_ = node_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "joint_angle_info", 10, std::bind(&ControlNode::JointAngle_callback, this, std::placeholders::_1));           // Wrapped

    protection_zone_subscriber_ = node_->create_subscription<kinova_msgs::msg::ProtectionZone>(
        "protection_zone", 10, std::bind(&ControlNode::protection_zone_callback, this, std::placeholders::_1));       // Wrapped

    estop_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
        "emergency_stop_cmd", 10, std::bind(&ControlNode::estop_callback, this, std::placeholders::_1));              // Wrapped

    fault_subscriber_ = node_->create_subscription<std_msgs::msg::Bool>(
        "clear_faults_cmd", 10, std::bind(&ControlNode::fault_callback, this, std::placeholders::_1));                // Wrapped

    gripper_subscriber_ = node_->create_subscription<control_msgs::msg::GripperCommand>(
        "gripper_command_cmd", 10, std::bind(&ControlNode::gripper_callback, this, std::placeholders::_1));           // Wrapped

    action_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "action", 10, std::bind(&ControlNode::action_callback, this, std::placeholders::_1));                         // Wrapped

    action_remover_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "delete_action", 10, std::bind(&ControlNode::action_remover_callback, this, std::placeholders::_1));          // Wrapped

    action_creater_subscriber_ = node_->create_subscription<kinova_msgs::msg::CreateAction>(
        "create_action", 10, std::bind(&ControlNode::action_creater_callback, this, std::placeholders::_1));          // Wrapped


    // Add shutdown hook to delete all protection zones created by messages
    rclcpp::on_shutdown([this]()
                        { this->delete_all_protection_zones(); });

    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base_->SetServoingMode(servoingMode);
  }
  
  static void signal_handler(int signal)
  {
    if (signal == SIGINT)
    {
      if (instance_)
      {
        instance_->delete_all_protection_zones();
      }
      rclcpp::shutdown();
    }
  }

  // Static function to delete protection zone for signal handling
  static void delete_all_protection_zones()
  {
    if (instance_)
    {
      for (const auto &handle : instance_->msg_protection_zone_handles_)
      {
        instance_->base_->DeleteProtectionZone(handle);
      }
    }
  }

private:
  /**
     * @brief Action(Preset) Creater Callback
     *        Create action(preset) by command
     * 
     * @param msg Action's information to create (Action(preset) name, each joint angles)
   */
  void action_creater_callback(const kinova_msgs::msg::CreateAction::SharedPtr msg)
  {
    k_api::Base::Action action;

    // Set Action(Preset) Name
    action.set_name(msg->name);

    // Set Action(Preset) Type
    action.set_application_data("");
    action.mutable_reach_joint_angles();

    // Set Target Joint Angles
    auto joint_angles = action.mutable_reach_joint_angles()->mutable_joint_angles();
    for (auto i = 0; i < msg->target_joint_angles.size(); i++)
    {
      auto joint_angle = joint_angles->add_joint_angles();
      joint_angle->set_joint_identifier(i);
      joint_angle->set_value(msg->target_joint_angles[i]);
    }

    // Create the action by using base Client
    k_api::Base::ActionHandle action_handle = base_->CreateAction(action);
    return;
  }

  /**
     * @brief Action(preset) Remover Callback
     * 
     * @param msg Action's name to delete (Action(Preset) name)
   */
  void action_remover_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto action_type = k_api::Base::RequestedActionType();
    auto action_list = base_->ReadAllActions(action_type);          // Read All kind of Action(Preset) saved in the robot
    action_type.set_action_type(k_api::Base::UNSPECIFIED_ACTION);   // All Kind of Action(Preset) Type
    auto action_handle = k_api::Base::ActionHandle();               // Create Action(Preset) Handle to delete

    for (auto action : action_list.action_list())
    {
      if (action.name() == msg->data)                               // Delete Action by searching by name
      {
        action_handle = action.handle();
        base_->DeleteAction(action_handle);
        return;
      }
    }
  }

  /**
     * @brief "move_to_preset_position" callback caller -> Do Selected Action
     * 
     * @param msg Action(Preset)'s name to execute
   */
  void action_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (estop_active_){
      return;
    }

    auto action_name = msg->data;

    // If Topic Msg contains Action's(Preset's) name..
    if (!msg->data.empty()){
      move_to_preset_position(base_, msg->data);    // Move to that Action(Preset)
    }
  }

    // When "Action_Callback" recieved action name to execute, found the action and play it. 
    bool move_to_preset_position(k_api::Base::BaseClient *base_, std::string preset)
      {
        try
        {
          k_api::Base::ArmStateInformation arm_state_info = base_->GetArmState();

          if (arm_state_info.active_state() == 4)
          {
            std::cout << "Robot is in fault state. Cannot send action command" << std::endl;
            return false;
          }

          // if (collision_detected.load()){
          //   std::cout << "Collision detected! Stopping robot." << std::endl;
          //   try{
          //     base_->Stop();
          //   }
          //   catch(k_api::KDetailedException &ex){
          //     if (ex.getErrorInfo().getError().error_sub_code() == k_api::SubErrorCodes::SESSION_NOT_IN_CONTROL){
          //       request_control();

          //       try
          //       {
          //         base_->Stop();
          //       }
          //       catch(const k_api::KDetailedException &ex)
          //       {
          //         RCLCPP_ERROR(node_->get_logger(), "Failed to stop the robot even after regaining control: %s", ex.what());
          //       }
          //     }

          //     else{
          //       RCLCPP_ERROR(node_->get_logger(), "An error occurred: %s", ex.what());
          //     }
          //   }

          //   return false;
          // }

          std::cout << "Moving the arm to " << preset << std::endl;
          auto action_type = k_api::Base::RequestedActionType();
          action_type.set_action_type(k_api::Base::UNSPECIFIED_ACTION); // Start by looking for all action types
          auto action_list = base_->ReadAllActions(action_type);        // Read(Get) All kind of Action(Preset)
          auto action_handle = k_api::Base::ActionHandle();

          action_handle.set_identifier(0);

          bool found_action = false;
          k_api::Base::ConstrainedJointAngles constrained_joint_angles;
          auto joint_angles = constrained_joint_angles.mutable_joint_angles();

          // Iterate through actions and find the one matching the preset name
          for (const auto &action : action_list.action_list())
          {
            if (action.name() == preset)
            {
              action_handle = action.handle();

              // Case 1: REACH_JOINT_ANGLES
              if (action.has_reach_joint_angles())
              {
                const auto &target_joint_angles = action.reach_joint_angles().joint_angles();
                for (int i = 0; i < target_joint_angles.joint_angles_size(); i++)
                {
                  auto joint_angle = joint_angles->add_joint_angles();
                  joint_angle->set_joint_identifier(i);
                  joint_angle->set_value(target_joint_angles.joint_angles(i).value());
                }
                found_action = true;
                break;
              } // End of Case 1 : Reach Joint Angles

              // Case 2: REACH_POSE (using Inverse Kinematics)
              else if (action.has_reach_pose())
              {
                // Get ConstrainedPose First
                const auto &constrained_pose = action.reach_pose();

                // Get target pose of Constrained pose
                const auto &pose = constrained_pose.target_pose();

                // Retrieve current joint angles
                k_api::Base::JointAngles current_joint_angles = base_->GetMeasuredJointAngles();

                // Set up IKData object with the target pose
                k_api::Base::IKData input_IkData;
                input_IkData.mutable_cartesian_pose()->set_x(pose.x());
                input_IkData.mutable_cartesian_pose()->set_y(pose.y());
                input_IkData.mutable_cartesian_pose()->set_z(pose.z());
                input_IkData.mutable_cartesian_pose()->set_theta_x(pose.theta_x());
                input_IkData.mutable_cartesian_pose()->set_theta_y(pose.theta_y());
                input_IkData.mutable_cartesian_pose()->set_theta_z(pose.theta_z());

                // Fill IKData with the current joint angle guess
                for (const auto &joint_angle : current_joint_angles.joint_angles())
                {
                  auto guessed_joint_angle = input_IkData.mutable_guess()->add_joint_angles();
                  guessed_joint_angle->set_value(joint_angle.value());
                }

                // Perform Inverse Kinematics
                try
                {
                  k_api::Base::JointAngles computed_joint_angles = base_->ComputeInverseKinematics(input_IkData);

                  for (const auto &joint_angle : computed_joint_angles.joint_angles())
                  {
                    auto new_joint_angle = joint_angles->add_joint_angles();
                    new_joint_angle->set_joint_identifier(joint_angle.joint_identifier());
                    new_joint_angle->set_value(joint_angle.value());
                  }
                  found_action = true;
                  break;
                }
                catch (const Kinova::Api::KDetailedException &e)
                {
                  std::cerr << "Failed to solve IK: " << e.what() << std::endl;
                  return false;
                }
              } // End of Case 2 : Reach Pose
            }
          }

          if (!found_action)
          {
            return false;
          }

          // Play the joint trajectory based on the computed joint angles
          base_mutex_.lock();
          base_->PlayJointTrajectory(constrained_joint_angles);
          base_mutex_.unlock();

          return true;
        }
        catch (const k_api::KDetailedException &ex)
        {
          std::cerr << "Caught Exception: " << ex.what() << std::endl;
          return false;
        }
      } // End of bool Move_To_Preset_Position

  // JointAngle Solver for GUI Input --> When GUI Send command of joint delta angle to move
  void JointAngle_callback(trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    k_api::Base::ArmStateInformation arm_state_info = base_->GetArmState();

    if (arm_state_info.active_state() == 4)
    {
      return;
    }    
    
    // if(collision_detected.load()){
    //   try{
    //     base_->Stop();
    //   }

    //   catch(k_api::KDetailedException &ex){
    //     if (ex.getErrorInfo().getError().error_sub_code() == k_api::SubErrorCodes::SESSION_NOT_IN_CONTROL){
    //       request_control();

    //       try
    //       {
    //         base_->Stop();
    //       }
    //       catch(const k_api::KDetailedException &ex)
    //       {
    //         RCLCPP_ERROR(node_->get_logger(), "Failed to stop the robot even after regaining control: %s", ex.what());
    //       }
    //     }

    //     else{
    //       RCLCPP_ERROR(node_->get_logger(), "An error occurred: %s", ex.what());
    //     }
    //   }      
    //   return;
    // }

    try
    {
      // Retrieve the current joint angles of the robot
      k_api::Base::JointAngles current_joint_angles  = base_->GetMeasuredJointAngles();
      k_api::Base::JointAngles updated_joint_angles;
      
      // Iterate through each joint to calculate the new angles
      for(int i = 0; i < current_joint_angles.joint_angles_size(); i++){
        // Get the current angle value of the Joints
        auto current_joint_angle = current_joint_angles.joint_angles(i);
        float delta_angle = (msg->points.size() > 0 && i < msg->points[0].positions.size()) ? msg->points[0].positions[i] : 0.0; // Delta Angle comes from Topic Msg
        float new_angle_value =  current_joint_angle.value() + delta_angle;                                                      // New Angle = current + delta

        // Add the new joint angle to the update_joint_angles object
        auto updated_joint_angle = updated_joint_angles.add_joint_angles();
        updated_joint_angle -> set_joint_identifier(current_joint_angle.joint_identifier());
        updated_joint_angle -> set_value(new_angle_value);
      }

      // Create a "Constrained_joint_angles" object to send the updated joint angles
      // Constrained_joint_angles -> allows to specify a target configuration for the robot's joints while ensuring that the motion remains within safe limits.
      k_api::Base::ConstrainedJointAngles constrained_joint_angles;
      auto joint_angles = constrained_joint_angles.mutable_joint_angles();

      // Transfer the updated joint angles to the constrained joint angle object
      for (const auto &joint_angle : updated_joint_angles.joint_angles()){
        auto new_joint_angle = joint_angles -> add_joint_angles();
        new_joint_angle -> set_joint_identifier(joint_angle.joint_identifier());
        new_joint_angle -> set_value(joint_angle.value());
      }

      base_->PlayJointTrajectory(constrained_joint_angles);
    }
    catch(const Kinova::Api::KDetailedException &e)
    {
      std::cerr << "Failed to move joint angle: " << e.what() << std::endl;
    }
  }

  // Updated Pose Calculator for GUI Input --> Not Wrapped in Wrapper T.B.D
  void EndEffector_callback(geometry_msgs::msg::Twist::SharedPtr msg)
  {
    k_api::Base::ArmStateInformation arm_state_info = base_->GetArmState();

    if (arm_state_info.active_state() == 4)
    {
      return;
    }

    // Get Current End Effector Pose and get ready to update it with delta pose from Topic Msg
    auto current_pose = base_->GetMeasuredCartesianPose();
    k_api::Base::ConstrainedPose constrained_pose;
    auto pose = constrained_pose.mutable_target_pose();

    // Set New Pose based on delta value.
    pose->set_x(current_pose.x() + msg->linear.x);
    pose->set_y(current_pose.y() + msg->linear.y);
    pose->set_z(current_pose.z() + msg->linear.z);
    pose->set_theta_x(current_pose.theta_x() + msg->angular.x);
    pose->set_theta_y(current_pose.theta_y() + msg->angular.y);
    pose->set_theta_z(current_pose.theta_z() + msg->angular.z);
      
    try
    {
      base_->PlayCartesianTrajectory(constrained_pose);
    }
    catch (const Kinova::Api::KDetailedException &e)
    {
      std::cerr << "Failed to solve IK: " << e.what() << std::endl;
    }
  }

   /**
     * @brief Protection zone Cmd by topic msg (Create Protection Zone & Delete Protection Zone by mode field)
     * 
     * @param msg Topic Msg that contains information about Protection Zone Cmd(3 modes) from Remote Controller.
     * 
     *  Modes
     *  1. mode 1 = Create Protection Zone (Zone Name, Shape type, is_enabled, origin, dimension, orientation, envelope thickness, target speed)
     *  2. mode 2 = Delete Protection Zone (Name)
     *  3. mode 3 = Update Protection Zone (Name, is_enabled)
   */
  void gripper_callback(const control_msgs::msg::GripperCommand::SharedPtr msg)
  {
    try
    {
      k_api::Base::ArmStateInformation arm_state_info = base_->GetArmState();

      if (arm_state_info.active_state() == 4)
      {
        return;
      }

      k_api::Base::GripperCommand gripper_command;
      gripper_command.set_mode(k_api::Base::GRIPPER_POSITION); // Control the Gripper movement with position

      auto finger = gripper_command.mutable_gripper()->add_finger();
      finger->set_finger_identifier(1);
      finger->set_value(msg->position);

      base_->SendGripperCommand(gripper_command);

      gripper_stop_ = false;
    }

    catch (const k_api::KDetailedException &ex)
    {
      return;
    }
  }

  // Clearing the Faults
  void fault_callback(const std_msgs::msg::Bool::SharedPtr)
  {
    base_->ClearFaults();

    // auto servoingMode = k_api::Base::ServoingModeInformation();
    // servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    // base_->SetServoingMode(servoingMode);

    gripper_stop_ = false;
    estop_active_ = false;
  }

  // Set Emergency Stop Control to Robot
  void estop_callback(const std_msgs::msg::Bool::SharedPtr)
  {
    base_->ApplyEmergencyStop(0, {true, 0, 1000});
    base_->ApplyEmergencyStop(0, {true, 0, 1000});
    estop_active_ = true;
  }

  // Send Twist Control by Joystick to the robot
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    try
    {
      k_api::Base::ArmStateInformation arm_state_info = base_->GetArmState();

      if (arm_state_info.active_state() == 4)
      {
        return;
      }

      twist_msg_ = *msg;

      if (estop_active_)
      {
        return;
      }

      auto command = k_api::Base::TwistCommand();
      command.set_reference_frame(k_api::Common::CARTESIAN_REFERENCE_FRAME_TOOL);

      // auto servoingMode = k_api::Base::ServoingModeInformation();
      // servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
      // base_->SetServoingMode(servoingMode);

      auto twist = command.mutable_twist();
      twist->set_linear_x(twist_msg_.linear.x);
      twist->set_linear_y(twist_msg_.linear.y);
      twist->set_linear_z(twist_msg_.linear.z);
      twist->set_angular_x(twist_msg_.angular.x);
      twist->set_angular_y(-1 * twist_msg_.angular.y);
      twist->set_angular_z(twist_msg_.angular.z);

      if (twist_msg_.linear.x == 0.0 && twist_msg_.linear.y == 0.0 &&
          twist_msg_.linear.z == 0.0 && twist_msg_.angular.x == 0.0 &&
          twist_msg_.angular.y == 0.0 && twist_msg_.angular.z == 0.0)
      {
        if (!is_released_)
        {
          base_->Stop();
          is_released_ = true;
        }
      }
      else
      {
        is_released_ = false;
        base_->SendTwistCommand(command);
      }
    }
    catch (const k_api::KDetailedException &ex)
    {
      is_released_ = true;
    }
  }

   /**
     * @brief Protection zone Cmd by topic msg (Create Protection Zone & Delete Protection Zone by mode field)
     * 
     * @param msg Topic Msg that contains information about Protection Zone Cmd(3 modes) from Remote Controller.
     * 
     *  Modes
     *  1. mode 1 = Create Protection Zone (Zone Name, Shape type, is_enabled, origin, dimension, orientation, envelope thickness, target speed)
     *  2. mode 2 = Delete Protection Zone (Name)
     *  3. mode 3 = Update Protection Zone (Name, is_enabled)
   */
  void protection_zone_callback(kinova_msgs::msg::ProtectionZone::SharedPtr msg)
  {
      k_api::Base::ProtectionZone zone;

      // Mode = 1 -> Create Protection Zone
      if(msg->mode == 1){
          zone.set_name(msg->name);                           // Protection Zone's name
          zone.set_is_enabled(msg->is_enabled);               // To Activate the zone or Deactivate the zone. (True = Activate, False = DeActivate)

          auto shape = zone.mutable_shape();
          shape->set_shape_type(static_cast<k_api::Base::ShapeType>(msg->shape.shape_type));  // Set Zone's shape. (CYLINDER = 1, SPHERE = 2, RECTANGULAR_PRISM = 3)

          // Set the Position for the Protection Zone to be located
          auto point = shape->mutable_origin();
          point->set_x(msg->shape.origin.x);
          point->set_y(msg->shape.origin.y);
          point->set_z(msg->shape.origin.z);

          // Set the Dimensions for the Protection Zone based on shape_type
          shape->clear_dimensions();

          // Catch the case of what kind of shaped Protection Zone that user want. (CYLINDER = 1, SPHERE = 2, RECTANGULAR_PRISM = 3)
          switch (static_cast<k_api::Base::ShapeType>(msg->shape.shape_type)) {
              case k_api::Base::ShapeType::CYLINDER: // CYLINDER
                  if (msg->shape.dimensions.size() >= 2) {
                    // dimensions: [radius, height]
                    shape->add_dimensions(msg->shape.dimensions[0]); // radius
                    shape->add_dimensions(msg->shape.dimensions[1]); // height
                  } 
                  else {
                    // Handle insufficient dimensions
                    float radius = (msg->shape.dimensions.size() >= 1) ? msg->shape.dimensions[0] : 0.0f;
                    shape->add_dimensions(radius); // radius
                    shape->add_dimensions(0.0f);   // height
                  }
                  break;
              case k_api::Base::ShapeType::SPHERE: // SPHERE
                  if (msg->shape.dimensions.size() >= 1) {
                    // dimensions: [radius]
                    shape->add_dimensions(msg->shape.dimensions[0]); // radius
                  } 
                  else {
                    // Set default radius
                    shape->add_dimensions(1.0f);
                  }
                  break;
              case k_api::Base::ShapeType::RECTANGULAR_PRISM: // RECTANGULAR_PRISM
                  if (msg->shape.dimensions.size() >= 3) {
                    // dimensions: [width, depth, height]
                    shape->add_dimensions(msg->shape.dimensions[0]); // width
                    shape->add_dimensions(msg->shape.dimensions[1]); // depth
                    shape->add_dimensions(msg->shape.dimensions[2]); // height
                  } 
                  else {
                      // Handle insufficient dimensions
                      for (size_t i = 0; i < 3; ++i) {
                        float dim = (i < msg->shape.dimensions.size()) ? msg->shape.dimensions[i] : 0.0f;
                        shape->add_dimensions(dim);
                      }
                  }
                  break;
              default:
                  return; 
          }

          // Set the Rotation for the Protection Zone (if applicable)
          if (msg->shape.shape_type != static_cast<uint32_t>(k_api::Base::ShapeType::SPHERE)) {
              auto orientation = shape->mutable_orientation();
              orientation->mutable_row1()->set_column1(msg->shape.orientation.row1[0]);
              orientation->mutable_row1()->set_column2(msg->shape.orientation.row1[1]);
              orientation->mutable_row1()->set_column3(msg->shape.orientation.row1[2]);

              orientation->mutable_row2()->set_column1(msg->shape.orientation.row2[0]);
              orientation->mutable_row2()->set_column2(msg->shape.orientation.row2[1]);
              orientation->mutable_row2()->set_column3(msg->shape.orientation.row2[2]);

              orientation->mutable_row3()->set_column1(msg->shape.orientation.row3[0]);
              orientation->mutable_row3()->set_column2(msg->shape.orientation.row3[1]);
              orientation->mutable_row3()->set_column3(msg->shape.orientation.row3[2]);
          }

          shape->set_envelope_thickness(msg->shape.envelope_thickness); // Thickness of the envelope --> Create enveloped_zone of outer_side

          // Add envelope limitations field for Protection Zone 
          for (const auto& env_limitation_msg : msg->envelope_limitations)
          {
              auto envelope_limitation = zone.mutable_envelope_limitations()->Add();
              envelope_limitation->set_type(static_cast<k_api::Base::LimitationType>(env_limitation_msg.type)); // Should match with Protection Zone's shape type
              envelope_limitation->set_translation(env_limitation_msg.translation); // Enveloped Zone's Speed Limit (m/s)
              envelope_limitation->set_orientation(env_limitation_msg.orientation);
          }

          k_api::Base::ProtectionZoneHandle protection_zone_handle = base_->CreateProtectionZone(zone);
          msg_protection_zone_handles_.push_back(protection_zone_handle);
      }

      // Mode = 2 -> Delete the selected Protection Zone. 
      else if(msg->mode == 2){
          const auto protection_zones = base_->ReadAllProtectionZones().protection_zones(); // Get All Protection zones that robot has.
          
          // Searching matched Protection Zone by name. 
          for(const auto &protection_zone : protection_zones){
              if(protection_zone.name() == msg->name){
                  base_->DeleteProtectionZone(protection_zone.handle());
                  std::cout<< "Deleted Protection Zone with name : " << msg->name << std::endl;
                  break;
              }
          }
      }

      // Mode = 3 -> Update is_enabled option for selected Protection Zone.
      else if(msg->mode == 3){
        const auto protection_zones = base_->ReadAllProtectionZones().protection_zones();

        // Matching Protection Zone by name.
        for (const auto &protection_zone : protection_zones){
            if(protection_zone.name() == msg->name){
                // Copy the existing protection zone
                k_api::Base::ProtectionZone zone = protection_zone;

                // Toggle the is_enabled flag
                if (!zone.is_enabled() == msg->is_enabled){                   // 
                  zone.set_is_enabled(msg->is_enabled);
                }

                // Update the protection zone with the modified zone
                base_->UpdateProtectionZone(zone);

                std::cout << "Updated Protection Zone '" << zone.name() << "' is_enabled to " << (zone.is_enabled() ? "true" : "false") << std::endl;

                break; // Protection zone found and updated, exit the loop
            }
        }
      }

  }

  // DeadZone Function
  float applyDeadzone(float value, float threshold = DEADZONE_THRESHOLD)
  {
    return (std::abs(value) < threshold) ? 0.0f : value;
  }

  // Use this function to retake the priority of control session when user lost control session priority to others.
  // Need = Session Manager, Create Session Info from main func.
  void request_control(){
   try
    {
      // Close current driver's session
      session_manager_->CloseSession();

      // Create a new session to retake the control priority
      session_manager_->CreateSession(create_session_info_);

      RCLCPP_INFO(node_->get_logger(), "Control has been re-acquired.");
    }
    catch (const k_api::KDetailedException &ex)
    {
      RCLCPP_ERROR(node_->get_logger(), "Failed to request control: %s", ex.what());
    }
    
  }

  std::shared_ptr<rclcpp::Node> node_;
  k_api::Base::BaseClient *base_;
  k_api::BaseCyclic::BaseCyclicClient *base_cyclic_;
  k_api::SessionManager *session_manager_;
  k_api::Session::CreateSessionInfo create_session_info_;
  static ControlNode *instance_;
  std::vector<k_api::Base::ProtectionZoneHandle> msg_protection_zone_handles_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr gui_end_effector_subscriber_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr gui_joint_angle_subscriber_;
  rclcpp::Subscription<kinova_msgs::msg::ProtectionZone>::SharedPtr protection_zone_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr fault_subscriber_;
  rclcpp::Subscription<control_msgs::msg::GripperCommand>::SharedPtr gripper_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_subscriber_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_remover_subscriber_;
  rclcpp::Subscription<kinova_msgs::msg::CreateAction>::SharedPtr action_creater_subscriber_;

  geometry_msgs::msg::Twist twist_msg_;
  bool gripper_stop_;
  bool estop_active_;
  bool is_released_{false};
  std::mutex base_mutex_;
};

ControlNode *ControlNode::instance_ = nullptr;

/**
 * @brief Chcek connection between TCP port and Robot
 * 
 * @param ip = Kinova Kortex's Robot Ip (fixed) = 192.168.1.10
 * @param port = Port 100000 (Defined in the header section)
 * @param timeout_sec = Maximum time trial to connect to TCP Port
 * @return true = Connection Succesful between the Robot and Driver
 * @return false = Connection Unsuccesful between the Robot and Driver
 */
bool isTcpPortOpen(const std::string& ip, int port, int timeout_sec) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if(sock < 0) {
        std::cerr << "Socket creation error: " << strerror(errno) << std::endl;
        return false;
    }

    // Set Socket as Nonblocking mode
    int flags = fcntl(sock, F_GETFL, 0);
    if(flags < 0) {
        std::cerr << "fcntl(F_GETFL) error: " << strerror(errno) << std::endl;
        close(sock);
        return false;
    }

    if(fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "fcntl(F_SETFL) error: " << strerror(errno) << std::endl;
        close(sock);
        return false;
    }

    struct sockaddr_in addr;
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    if(inet_pton(AF_INET, ip.c_str(), &addr.sin_addr) <= 0) {
        std::cerr << "Invalid IP address: " << ip << std::endl;
        close(sock);
        return false;
    }

    int ret = connect(sock, (struct sockaddr*)&addr, sizeof(addr));
    if(ret < 0) {
        if(errno != EINPROGRESS) {
            // Failure on Connection
            close(sock);
            return false;
        }
    }

    fd_set fdset;
    struct timeval tv;

    FD_ZERO(&fdset);
    FD_SET(sock, &fdset);
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;

    // Use Select to check possibility to connect
    ret = select(sock + 1, NULL, &fdset, NULL, &tv);
    if(ret > 0) {
        int so_error;
        socklen_t len = sizeof(so_error);

        getsockopt(sock, SOL_SOCKET, SO_ERROR, &so_error, &len);
        if(so_error == 0) {
            // Succeed in connection
            close(sock);
            return true;
        } else {
            // Failed in connection
            close(sock);
            return false;
        }
    } else if(ret == 0) {
        // Timeout
        close(sock);
        return false;
    } else {
        // select error
        std::cerr << "select error: " << strerror(errno) << std::endl;
        close(sock);
        return false;
    }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Parse command-line arguments(IP address, username, password)
  // IP Address : 192.168.1.10
  // username : admin
  // password : admin
  auto parsed_args = ParseExampleArguments(argc, argv);

  bool ping_success = false;

  while(rclcpp::ok() && !ping_success) {
      // Create Ping Command (ICMP packey 1, timeout 1s)
      std::string command = "ping -c 1 -W 1 " + parsed_args.ip_address + " > /dev/null 2>&1";

      // Run System Command
      int ret = std::system(command.c_str());

      if(ret == 0) {
          std::cout << "Ping to " << parsed_args.ip_address << " succeeded." << std::endl;
          ping_success = true;
      } else {
          std::cout << "Ping to " << parsed_args.ip_address << " failed. Retrying in 3 seconds..." << std::endl;
          std::this_thread::sleep_for(std::chrono::seconds(3));
      }
  }


int port = 10000; // Robot's TCP port num 10000
int timeout_sec = 1; // Timeout Setting
bool port_open = false;

while(rclcpp::ok() && !port_open) {
    if(isTcpPortOpen(parsed_args.ip_address, port, timeout_sec)) {
        std::cout << "TCP port " << port << " on " << parsed_args.ip_address << " is open." << std::endl;
        port_open = true;
    } else {
        std::cout << "Cannot connect to TCP port " << port << " on " << parsed_args.ip_address << ". Retrying in 3 seconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }
}

  // Create API objects to communicate with Kinova Kortex gen 3
  auto error_callback = [](k_api::KError err)
  { cout << "_________ callback error _________" << err.toString(); };  // Error Handling Callback

  // Transport layer = Establish TCP connection to the robot
  auto transport = new k_api::TransportClientTcp();
  auto router = new k_api::RouterClient(transport, error_callback);
  transport->connect(parsed_args.ip_address, PORT);                     // Connect to the robot using parsed IP address and port  

  // Set session data connection information
  auto create_session_info = k_api::Session::CreateSessionInfo();
  create_session_info.set_username(parsed_args.username);
  create_session_info.set_password(parsed_args.password);
  create_session_info.set_session_inactivity_timeout(60000);            // (milliseconds)
  create_session_info.set_connection_inactivity_timeout(2000);          // (milliseconds)

  // Session manager service wrapper : Manage the session with the robot
  std::cout << "Creating session for communication" << std::endl;
  auto session_manager = new k_api::SessionManager(router);
  session_manager->CreateSession(create_session_info);
  std::cout << "Session created" << std::endl;

  // Create Service clients for Base and BaseCyclic APIs
  auto base = new k_api::Base::BaseClient(router);
  bool ready_to_use = false;

  /**
   * @brief 40seconds needed to ready the robot. When Robot is booting on, arm state is changing 3 -> 7
   *  Arm State 3: Initialization
   *            7: Servoing Ready
   *  When Armstate begun "7(Servoing Ready)" driver is able to connect between the Robot and Driver 
   */
  while(rclcpp::ok() && !(base->GetArmState().active_state() == 7)){
      std::cout << "robot is not ready to use. \n";
  }
  std::cout << "robot is ready to use. \n";


  auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router);

  // Create Two nodes "Publishing" Robot's State & control after "Subscribing" Remote Controller's Cmd. 
  auto pub_node = rclcpp::Node::make_shared("status_node");
  auto sub_node = rclcpp::Node::make_shared("control_node");

  // Pass base_cyclic as the third argument for ControlNode
  auto status_node = std::make_shared<StatusNode>(pub_node, base, base_cyclic);
  auto control_node = std::make_shared<ControlNode>(sub_node, base, base_cyclic, session_manager, create_session_info);

  std::cout << "Kinova Driver Started" << std::endl;

  // Create a MultiThreadedExecutor with 4 threads for handling callbacks
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);

  // Add nodes to Executor
  executor.add_node(pub_node);
  executor.add_node(sub_node);

  // Spin the callback
  executor.spin();

  // --- Cleaning UP from here---

  // Not using this Function For now
  /*
    Further Use:
      1. When User want to create a Protection Zone when start Kinova_Driver, Fix the dimensions, Origin, Orientation at first
      2. Push back that zone to handle
      3. When turn off the Kinova_Driver, this function will clean up all the protection zone which created by this driver, not by Remote Controller's Callback.
  */
  control_node->delete_all_protection_zones();

  // Close the session for the robot
  session_manager->CloseSession();

  // Deactivate the router and cleanly disconnect from the transport object
  router->SetActivationStatus(false);
  transport->disconnect();

  // Destroy the API -> Release dynamically allocated memory
  delete base;
  delete session_manager;
  delete router;
  delete transport;

  return 0;
}
