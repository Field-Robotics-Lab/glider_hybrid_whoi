cmake_minimum_required(VERSION 2.8.3)
project(direct_kinematics_gazebo_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    gazebo_dev)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

set(DIRECT_KINEMATICS_GAZEBO_PLUGINS_LIST "")

catkin_package(
  LIBRARIES
    direct_kinematics
)

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${catkin_INCLUDE_DIRS}
                    ${GAZEBO_INCLUDE_DIRS}
                    ${roscpp_INCLUDE_DIRS}
                    ${std_msgs_INCLUDE_DIRS})

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
  ${roscpp_LIBRARIES})

###########
## Build ##
###########

add_library(direct_kinematics
    SHARED
    src/direct_kinematics.cc
)
list(APPEND DIRECT_KINEMATICS_GAZEBO_PLUGINS_LIST direct_kinematics)

#############
## Install ##
#############

install(TARGETS ${DIRECT_KINEMATICS_GAZEBO_PLUGINS_LIST}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install scripts files
install(DIRECTORY scripts
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)