cmake_minimum_required(VERSION 2.8.3)
project(voice_recognition)

find_package(catkin
  REQUIRED COMPONENTS
  rospy 
  roscpp 
  roslib
  actionlib

  message_generation
  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  trajectory_msgs
  baxter_core_msgs
  geometry_msgs
  skeletonmsgs_nu

  cv_bridge
  baxter_interface
  nxr_baxter
  tf
  dynamic_reconfigure
)

catkin_python_setup()

add_message_files(
  FILES
  memory.msg
)

generate_messages(DEPENDENCIES std_msgs std_srvs sensor_msgs geometry_msgs control_msgs trajectory_msgs baxter_core_msgs skeletonmsgs_nu)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  roslib
  actionlib 

  std_msgs
  std_srvs
  sensor_msgs
  control_msgs
  trajectory_msgs
  baxter_core_msgs
  geometry_msgs
  skeletonmsgs_nu

  cv_bridge
  baxter_interface
  nxr_baxter
  tf
  dynamic_reconfigure
)
