cmake_minimum_required(VERSION 2.8.3)
project(villa_manipulation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  actionlib
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  control_msgs
  message_generation
  geometry_msgs
  sensor_msgs
  tmc_yolo2_ros
  activity_detection
)

catkin_python_setup()

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Pickup.srv
   BroadcastObjectFrames.srv
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  trajectory_msgs
  control_msgs
  geometry_msgs
  sensor_msgs
  tmc_yolo2_ros
  activity_detection
)

install(DIRECTORY scripts/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

