from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _is_true(context, name):
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _validate_opt_in(context):
    local_enabled = _is_true(context, "enable_local_ekf")
    navsat_enabled = _is_true(context, "enable_navsat_transform")
    global_enabled = _is_true(context, "enable_global_ekf")
    if local_enabled and not _is_true(context, "local_inputs_verified"):
        raise RuntimeError("LOCALIZATION_NOT_READY: local_inputs_verified is false")
    if (navsat_enabled or global_enabled) and not _is_true(
        context, "yaw_convention_verified"
    ):
        raise RuntimeError("LOCALIZATION_NOT_READY: yaw_convention_verified is false")
    if navsat_enabled and not local_enabled:
        raise RuntimeError("LOCALIZATION_NOT_READY: navsat requires local EKF")
    if global_enabled and not navsat_enabled:
        raise RuntimeError("LOCALIZATION_NOT_READY: global EKF requires navsat")
    return []


def generate_launch_description():
    share = FindPackageShare("module_robot_localization")
    use_sim_time = LaunchConfiguration("use_sim_time")

    local_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="local_ekf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_local_ekf")),
        parameters=[
            PathJoinSubstitution([share, "config", "ekf_local.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
    )
    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_navsat_transform")),
        parameters=[
            PathJoinSubstitution([share, "config", "navsat_transform.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("imu/data", "/imu/data"),
            ("gps/fix", "/gps/fix"),
            ("odometry/filtered", "/odometry/filtered"),
            ("odometry/gps", "/odometry/gps"),
        ],
    )
    global_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="global_ekf",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_global_ekf")),
        parameters=[
            PathJoinSubstitution([share, "config", "ekf_global.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("odometry/filtered", "/odometry/global")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("local_inputs_verified", default_value="false"),
            DeclareLaunchArgument("yaw_convention_verified", default_value="false"),
            DeclareLaunchArgument(
                "enable_local_ekf",
                default_value="false",
                description="Opt in only after motor odometry and IMU rate checks",
            ),
            DeclareLaunchArgument(
                "enable_navsat_transform",
                default_value="false",
                description="Opt in only after yaw convention is proven",
            ),
            DeclareLaunchArgument(
                "enable_global_ekf",
                default_value="false",
                description="Opt in only after navsat_transform validation",
            ),
            OpaqueFunction(function=_validate_opt_in),
            local_ekf,
            navsat,
            global_ekf,
        ]
    )
