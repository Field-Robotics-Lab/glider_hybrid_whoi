cmake_minimum_required(VERSION 2.8.3)
project(kinematics_ros_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  geometry_msgs
  kinematics_plugins
  kinematics_ros_plugins_msgs
  gazebo_dev)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
  kinematics_ros_plugin
  kinematics_sc_ros_interface_plugin
 CATKIN_DEPENDS
   roscpp
   gazebo_msgs
   geometry_msgs
   kinematics_plugins
   kinematics_ros_plugins_msgs
   gazebo_dev
)

include_directories(
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_library(kinematics_ros_plugin src/KinematicsROSPlugin.cc)
target_link_libraries(kinematics_ros_plugin ${catkin_LIBRARIES})
add_dependencies(kinematics_ros_plugin ${catkin_EXPORTED_TARGETS})

add_library(kinematics_sc_ros_interface_plugin src/SphericalCoordinatesROSInterfacePlugin.cc)
target_link_libraries(kinematics_sc_ros_interface_plugin ${catkin_LIBRARIES})
add_dependencies(kinematics_sc_ros_interface_plugin ${catkin_EXPORTED_TARGETS})

## Mark executables and/or libraries for installation
install(TARGETS
    kinematics_ros_plugin
    kinematics_sc_ros_interface_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".~" EXCLUDE
)
