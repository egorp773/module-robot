from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("module_robot_description"), "urdf", "module_robot.urdf.xacro"]
    )
    argument_names = (
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
    declarations = [
        DeclareLaunchArgument(
            name,
            default_value="0.0",
            description=f"Measured value for {name}; zero is the uncalibrated TF-only default",
        )
        for name in argument_names
    ]
    xacro_arguments = []
    for name in argument_names:
        xacro_arguments.extend([f" {name}:=", LaunchConfiguration(name)])

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file, *xacro_arguments]),
        value_type=str,
    )
    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": False}],
    )
    return LaunchDescription([*declarations, publisher])
