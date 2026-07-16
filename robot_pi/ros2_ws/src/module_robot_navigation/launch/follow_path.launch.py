from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")

    controller = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[("cmd_vel", "/cmd_vel_nav_raw")],
    )
    smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel", "/cmd_vel_nav_raw"),
            ("cmd_vel_smoothed", "/cmd_vel_nav"),
        ],
    )
    lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_follow_path",
        output="screen",
        parameters=[
            params_file,
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": ["controller_server", "velocity_smoother"],
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("module_robot_navigation"),
                        "config",
                        "nav2_follow_path.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="false",
                description="Must remain false until autonomous preflight gates pass",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            controller,
            smoother,
            lifecycle,
        ]
    )
