cmake_minimum_required(VERSION 2.8.3)
project(direct_kinematics_ros_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  gazebo_msgs
  std_msgs
  roscpp
  tf2_ros
  tf2
  gazebo_dev
  frl_vehicle_msgs
  gazebo_ros
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
 CATKIN_DEPENDS
  geometry_msgs
  gazebo_msgs
  frl_vehicle_msgs
  std_msgs
  gazebo_ros
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(direct_kinematics_ros_plugin src/DirectKinematicsROSPlugin.cc)
target_link_libraries(direct_kinematics_ros_plugin ${catkin_LIBRARIES})
add_dependencies(direct_kinematics_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST direct_kinematics_ros_plugin)

install(TARGETS ${hybrid_glider_GAZEBO_ROS_PLUGINS_LIST}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hh"
  PATTERN "*~" EXCLUDE
)
