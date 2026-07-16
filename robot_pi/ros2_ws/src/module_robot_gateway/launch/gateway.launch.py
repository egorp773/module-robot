from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("module_robot_gateway"), "config", "gateway.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="module_robot_gateway",
                executable="gateway_node",
                name="gateway_node",
                output="screen",
                emulate_tty=True,
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
