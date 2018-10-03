cmake_minimum_required(VERSION 2.8.3)
project(pa10_controller_interface)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  pa10controller
  roscpp
  rospy
  sensor_msgs
  std_msgs
  trajectory_msgs 
  industrial_robot_client
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy pa10controller std_msgs sensor_msgs control_msgs trajectory_msgs industrial_robot_client
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS pa10_controller_interface DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)