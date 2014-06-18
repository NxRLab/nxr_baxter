cmake_minimum_required(VERSION 2.8.3)
project(nxr_baxter)

find_package(catkin REQUIRED
  COMPONENTS
  rospy 
  roscpp 
  roslib
  actionlib

  message_generation
  sensor_msgs
  std_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  baxter_core_msgs
  skeletonmsgs_nu
  nxr_baxter_msgs
  
  baxter_interface

  tf
  dynamic_reconfigure
  cv_bridge

)

include_directories(${catkin_INCLUDE_DIRS})

catkin_python_setup()

# generate_messages(DEPENDENCIES sensor_msgs std_msgs geometry_msgs control_msgs trajectory_msgs baxter_core_msgs skeletonmsgs_nu)

catkin_package(
  CATKIN_DEPENDS
  rospy
  roscpp
  roslib
  actionlib

  sensor_msgs
  std_msgs
  geometry_msgs
  control_msgs
  trajectory_msgs
  baxter_core_msgs
  skeletonmsgs_nu
  message_runtime
  nxr_baxter_msgs
  
  baxter_interface

  tf
  dynamic_reconfigure
  cv_bridge
)

add_executable(cpp_test_moveit src/cpp_test_moveit.cpp)
