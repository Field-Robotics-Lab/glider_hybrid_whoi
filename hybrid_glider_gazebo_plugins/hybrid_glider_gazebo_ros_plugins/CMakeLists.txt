cmake_minimum_required(VERSION 2.8.3)
project(hybrid_glider_gazebo_ros_plugins)

# Specify C++11 standard
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hybrid_glider_gazebo_plugins
  sensor_msgs
  geometry_msgs
  std_msgs
  hybrid_glider_gazebo_ros_plugins_msgs
  roscpp
  visualization_msgs
  tf2_ros
  tf2
  gazebo_dev
)


set(HYBRID_GLIDER_GAZEBO_ROS_PLUGINS_LIST "")

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES
  hybrid_glider_fin_ros_plugin
  hybrid_glider_thruster_ros_plugin
  hybrid_glider_underwater_object_ros_plugin
  hybrid_glider_joint_state_publisher
  hybrid_glider_accelerations_test_plugin
 CATKIN_DEPENDS hybrid_glider_gazebo_plugins
  sensor_msgs
  geometry_msgs
  std_msgs
  hybrid_glider_gazebo_ros_plugins_msgs
  visualization_msgs
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

add_library(hybrid_glider_fin_ros_plugin src/FinROSPlugin.cc)
target_link_libraries(hybrid_glider_fin_ros_plugin ${catkin_LIBRARIES})
add_dependencies(hybrid_glider_fin_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_fin_ros_plugin)

add_library(hybrid_glider_thruster_ros_plugin src/ThrusterROSPlugin.cc)
target_link_libraries(hybrid_glider_thruster_ros_plugin ${catkin_LIBRARIES})
add_dependencies(hybrid_glider_thruster_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_thruster_ros_plugin)

add_library(hybrid_glider_underwater_object_ros_plugin src/UnderwaterObjectROSPlugin.cc)
target_link_libraries(hybrid_glider_underwater_object_ros_plugin ${catkin_LIBRARIES})
add_dependencies(hybrid_glider_underwater_object_ros_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_underwater_object_ros_plugin)

add_library(hybrid_glider_joint_state_publisher src/JointStatePublisher.cc)
target_link_libraries(hybrid_glider_joint_state_publisher ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_joint_state_publisher)

add_library(hybrid_glider_accelerations_test_plugin src/AccelerationsTestPlugin.cc)
target_link_libraries(hybrid_glider_accelerations_test_plugin ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_accelerations_test_plugin)

add_library(hybrid_glider_linear_battery_ros_plugin src/LinearBatteryROSPlugin.cc)
target_link_libraries(hybrid_glider_linear_battery_ros_plugin LinearBatteryPlugin ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_linear_battery_ros_plugin)

add_library(hybrid_glider_custom_battery_consumer_ros_plugin src/CustomBatteryConsumerROSPlugin.cc)
target_link_libraries(hybrid_glider_custom_battery_consumer_ros_plugin ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST hybrid_glider_custom_battery_consumer_ros_plugin)

add_library(uw_gps_ros_plugin src/UnderwaterGPSROSPlugin.cc)
target_link_libraries(uw_gps_ros_plugin  ${catkin_LIBRARIES})
list(APPEND hybrid_glider_GAZEBO_ROS_PLUGINS_LIST uw_gps_ros_plugin)


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

# Install urdf files
install(DIRECTORY urdf
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(xacro REQUIRED)
  find_package(rosunit REQUIRED)
  foreach(T
      test/test_default_fossen_vehicle.test
      test/test_sphere_vehicle.test
      test/test_thrusters.test
      )
     add_rostest(${T})
  endforeach()
endif()
