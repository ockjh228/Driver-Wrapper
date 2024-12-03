from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # Initialize Arguments
    # robot_ip = LaunchConfiguration("robot_ip")
    # use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    # gripper_max_velocity = LaunchConfiguration("gripper_max_velocity")
    # gripper_max_force = LaunchConfiguration("gripper_max_force")
    # launch_rviz = LaunchConfiguration("launch_rviz")
    # use_sim_time = LaunchConfiguration("use_sim_time")
    # use_internal_bus_gripper_comm = LaunchConfiguration("use_internal_bus_gripper_comm")

    launch_arguments = {
        "robot_ip": "192.168.1.10",
        "use_fake_hardware": "false",
        "gripper": "robotiq_2f_85",
        "gripper_joint_name": "robotiq_85_left_knuckle_joint",
        "dof": "6",
        "gripper_max_velocity": "10",
        "gripper_max_force": "10",
        "use_internal_bus_gripper_comm": "false",
    }
    
    moveit_config = (
        MoveItConfigsBuilder("gen3", package_name="kinova_gen3_6dof_robotiq_2f_85_moveit_config")
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    
    joint_node = Node(
        package="joint_pkg",
        executable="joint_pub",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )
    pose_node = Node(
        package="joint_pkg",
        executable="pose_pub",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],    
    )    
    # user_input_node = Node(
    #     package="joint_pkg",
    #     executable="user_input",
    #     output="screen",
    #     parameters=[
    #         # {"allow_repeat": False},
    #         # {"repeat_delay": 500},
    #         # {"repeat_interval": 30},
    #     ],    
    # )    

    return LaunchDescription([
        joint_node,
        pose_node,
        # user_input_node,
    ])