from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python import get_package_share_directory


def generate_launch_description():
    package_name = "manipulator_wrapper"
    share_directory_path = get_package_share_directory(package_name)
    return LaunchDescription(
        [
            Node(
                package=package_name,
                namespace=package_name,
                executable=package_name,
                name=package_name,
                arguments=[
                    share_directory_path + "/config/parameter.yaml",
                    share_directory_path + "/log",
                ],
                output="screen",
            ),
        ]
    )
