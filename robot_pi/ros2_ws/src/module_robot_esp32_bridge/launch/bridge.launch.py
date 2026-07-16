from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    default_config = PathJoinSubstitution(
        [FindPackageShare("module_robot_esp32_bridge"), "config", "bridge.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="module_robot_esp32_bridge",
                executable="esp32_bridge",
                name="esp32_bridge",
                output="screen",
                parameters=[LaunchConfiguration("config")],
                emulate_tty=True,
            ),
        ]
    )
