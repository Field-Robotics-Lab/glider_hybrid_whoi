cmake_minimum_required(VERSION 2.8.3)
project(glider_hybrid_whoi_ros_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  geometry_msgs
  std_msgs
  roscpp
  visualization_msgs
  tf2_ros
  tf2
  gazebo_dev
  hector_gazebo_plugins
)


set(HYBRID_GLIDER_GAZEBO_ROS_PLUGINS_LIST "")

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
  sensor_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  hector_gazebo_plugins
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

add_library(uw_gps_ros_plugin src/UnderwaterGPSROSPlugin.cc)
target_link_libraries(uw_gps_ros_plugin  ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST uw_gps_ros_plugin)

add_library(uw_imu_ros_plugin src/UnderwaterIMUROSPlugin.cc)
target_link_libraries(uw_imu_ros_plugin  ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST uw_imu_ros_plugin)


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