from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_node",
                default_value="false",
                description="Only starts the disabled-by-parameters skeleton",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("module_robot_localization"),
                        "config",
                        "heading_initializer.yaml",
                    ]
                ),
            ),
            Node(
                package="module_robot_localization",
                executable="module_robot_heading_initializer",
                name="heading_initializer",
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_node")),
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
