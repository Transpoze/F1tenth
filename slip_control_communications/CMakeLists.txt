cmake_minimum_required(VERSION 2.8.3)
project(slip_control_communications)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  out_pursuit.msg
  input_drive.msg
  input_model.msg
  input_pid.msg
  lidar_data.msg
  mocap_data.msg
  pose.msg
  pose_and_references.msg
  )

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
    message_generation
    message_runtime
    std_msgs
    geometry_msgs
    sensor_msgs
  )
