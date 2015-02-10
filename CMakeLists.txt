cmake_minimum_required(VERSION 2.8.3)
project(nxr_baxter)

find_package(catkin
  REQUIRED
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
  baxter_moveit_config

  moveit_ros_planning_interface

  tf
  dynamic_reconfigure
  cv_bridge

)

find_package(Boost REQUIRED system filesystem date_time thread)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_INCLUDE_DIR})

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_definitions(${EIGEN_DEFINITIONS})

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
  baxter_moveit_config

  tf
  dynamic_reconfigure
  cv_bridge
  DEPENDS eigen
)


add_executable(kinematics_tester src/kinematic_tester.cpp)
# add_dependencies(skeletontracker skeletonmsgs_nu_gencpp)

target_link_libraries(kinematics_tester ${catkin_LIBRARIES} ${Boost_LIBRARIES})



