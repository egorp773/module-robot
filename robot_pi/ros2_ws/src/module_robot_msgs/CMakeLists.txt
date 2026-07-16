cmake_minimum_required(VERSION 3.8)
project(module_robot_msgs)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/RobotStatus.msg"
  "msg/MotorStatus.msg"
  "msg/RtkStatus.msg"
  "msg/PowerStatus.msg"
  "msg/RelayStatus.msg"
  "msg/ProtocolStats.msg"
  "msg/SafetyState.msg"
  "msg/FaultEvent.msg"
)

set(srv_files
  "srv/Arm.srv"
  "srv/Disarm.srv"
  "srv/ResetFault.srv"
  "srv/ResetEstop.srv"
  "srv/SetRelays.srv"
  "srv/SetControlMode.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
