from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from module_robot_bringup.preflight import (
    PreflightError,
    resolve_dimensions,
    validate_dimensions,
)


def _is_true(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in {
        "1",
        "true",
        "yes",
        "on",
    }


def _preflight_and_expand(context):
    if not _is_true(context, "allow_autonomy"):
        raise RuntimeError("AUTO_DISABLED: pass allow_autonomy:=true explicitly")
    if not _is_true(context, "manual_control_verified"):
        raise RuntimeError("AUTO_NOT_READY: manual_control_verified is false")
    if not _is_true(context, "watchdog_stop_verified"):
        raise RuntimeError("AUTO_NOT_READY: watchdog_stop_verified is false")
    if not _is_true(context, "yaw_convention_verified"):
        raise RuntimeError("AUTO_NOT_READY: yaw_convention_verified is false")
    if not _is_true(context, "localization_verified"):
        raise RuntimeError("AUTO_NOT_READY: localization_verified is false")

    dimensions_file = LaunchConfiguration("dimensions_file").perform(context)
    try:
        dimensions = resolve_dimensions(validate_dimensions(dimensions_file))
    except PreflightError as exc:
        raise RuntimeError(f"AUTO_NOT_READY: {exc}") from exc

    description_arguments = {
        "track_width_m": str(dimensions["track_width_m"]),
        "robot_length_m": str(dimensions["robot_length_m"]),
        "robot_width_m": str(dimensions["robot_width_m"]),
        "robot_height_m": str(dimensions["robot_height_m"]),
        "gps_x_m": str(dimensions["gps_x_m"]),
        "gps_y_m": str(dimensions["gps_y_m"]),
        "gps_z_m": str(dimensions["gps_z_m"]),
        "imu_x_m": str(dimensions["imu_x_m"]),
        "imu_y_m": str(dimensions["imu_y_m"]),
        "imu_z_m": str(dimensions["imu_z_m"]),
        "imu_roll": str(dimensions["imu_roll_rad"]),
        "imu_pitch": str(dimensions["imu_pitch_rad"]),
        "imu_yaw": str(dimensions["imu_yaw_rad"]),
    }
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("module_robot_description"),
                    "launch",
                    "description.launch.py",
                ]
            )
        ),
        launch_arguments=description_arguments.items(),
    )
    manual = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("module_robot_bringup"),
                    "launch",
                    "manual_bringup.launch.py",
                ]
            )
        ),
        launch_arguments={
            "start_bridge": LaunchConfiguration("start_bridge"),
            "start_description": "false",
            "start_gateway": LaunchConfiguration("start_gateway"),
            "start_tools": "true",
            "autonomous": "false",
            "bridge_config": LaunchConfiguration("bridge_config"),
            "safety_config": LaunchConfiguration("safety_config"),
            "gateway_config": LaunchConfiguration("gateway_config"),
        }.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("module_robot_localization"),
                    "launch",
                    "localization_templates.launch.py",
                ]
            )
        ),
        launch_arguments={
            "enable_local_ekf": "true",
            "enable_navsat_transform": "true",
            "enable_global_ekf": "true",
            "local_inputs_verified": "true",
            "yaw_convention_verified": "true",
            "use_sim_time": "false",
        }.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("module_robot_navigation"),
                    "launch",
                    "follow_path.launch.py",
                ]
            )
        ),
        launch_arguments={
            "autostart": "true",
            "use_sim_time": "false",
            "params_file": LaunchConfiguration("nav2_params_file"),
        }.items(),
    )
    return [
        LogInfo(
            msg=(
                "Static autonomous preflight passed. No ARM and no FollowPath goal "
                "is issued by bringup; live Safety gates still apply."
            )
        ),
        description,
        manual,
        localization,
        navigation,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("allow_autonomy", default_value="false"),
            DeclareLaunchArgument("manual_control_verified", default_value="false"),
            DeclareLaunchArgument("watchdog_stop_verified", default_value="false"),
            DeclareLaunchArgument("yaw_convention_verified", default_value="false"),
            DeclareLaunchArgument("localization_verified", default_value="false"),
            DeclareLaunchArgument("start_bridge", default_value="true"),
            DeclareLaunchArgument("start_gateway", default_value="false"),
            DeclareLaunchArgument(
                "bridge_config",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("module_robot_esp32_bridge"),
                        "config",
                        "bridge.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "safety_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("module_robot_safety"), "config", "safety.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "gateway_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("module_robot_gateway"), "config", "gateway.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "dimensions_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("module_robot_description"),
                        "config",
                        "robot_dimensions.yaml",
                    ]
                ),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare("module_robot_navigation"),
                        "config",
                        "nav2_follow_path.yaml",
                    ]
                ),
            ),
            OpaqueFunction(function=_preflight_and_expand),
        ]
    )
