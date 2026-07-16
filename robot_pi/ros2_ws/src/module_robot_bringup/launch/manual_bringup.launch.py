from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _reject_autonomous_alias(context):
    value = LaunchConfiguration("autonomous").perform(context).strip().lower()
    if value in {"1", "true", "yes", "on"}:
        raise RuntimeError(
            "manual_bringup never enables autonomy; use autonomous_bringup.launch.py "
            "and satisfy its explicit preflight gates"
        )
    return []


def generate_launch_description():
    bridge_config = PathJoinSubstitution(
        [FindPackageShare("module_robot_esp32_bridge"), "config", "bridge.yaml"]
    )
    safety_config = PathJoinSubstitution(
        [FindPackageShare("module_robot_safety"), "config", "safety.yaml"]
    )
    gateway_config = PathJoinSubstitution(
        [FindPackageShare("module_robot_gateway"), "config", "gateway.yaml"]
    )

    dimension_arguments = (
        "track_width_m",
        "robot_length_m",
        "robot_width_m",
        "robot_height_m",
        "gps_x_m",
        "gps_y_m",
        "gps_z_m",
        "imu_x_m",
        "imu_y_m",
        "imu_z_m",
        "imu_roll",
        "imu_pitch",
        "imu_yaw",
    )
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
        launch_arguments={
            name: LaunchConfiguration(name) for name in dimension_arguments
        }.items(),
        condition=IfCondition(LaunchConfiguration("start_description")),
    )
    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("module_robot_esp32_bridge"),
                    "launch",
                    "bridge.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("start_bridge")),
        launch_arguments={"config": LaunchConfiguration("bridge_config")}.items(),
    )
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("module_robot_safety"),
                    "launch",
                    "safety.launch.py",
                ]
            )
        ),
        launch_arguments={"config": LaunchConfiguration("safety_config")}.items(),
    )
    gateway = Node(
        package="module_robot_gateway",
        executable="gateway_node",
        name="gateway_node",
        output="screen",
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration("start_gateway")),
        parameters=[LaunchConfiguration("gateway_config")],
    )
    tools = Node(
        package="module_robot_tools",
        executable="readiness_monitor",
        name="readiness_monitor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_tools")),
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("module_robot_tools"), "config", "readiness.yaml"]
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_bridge",
                default_value="true",
                description="Set false in systemd because the bridge has its own service",
            ),
            DeclareLaunchArgument(
                "start_description",
                default_value="false",
                description=(
                    "Keep false while sensor/chassis transforms are TODO_MEASURE; "
                    "the serial bridge and manual Safety path do not require TF"
                ),
            ),
            DeclareLaunchArgument("start_gateway", default_value="false"),
            DeclareLaunchArgument("start_tools", default_value="true"),
            DeclareLaunchArgument(
                "autonomous",
                default_value="false",
                description="Compatibility argument; true is rejected by this manual launch",
            ),
            DeclareLaunchArgument("bridge_config", default_value=bridge_config),
            DeclareLaunchArgument("safety_config", default_value=safety_config),
            DeclareLaunchArgument("gateway_config", default_value=gateway_config),
            *[
                DeclareLaunchArgument(
                    name,
                    default_value="0.0",
                    description="Measured TF value; zero is allowed for manual TF skeleton",
                )
                for name in dimension_arguments
            ],
            OpaqueFunction(function=_reject_autonomous_alias),
            description,
            bridge,
            safety,
            gateway,
            tools,
        ]
    )
