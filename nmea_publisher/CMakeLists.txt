cmake_minimum_required(VERSION 3.0.2)
project(nmea_publisher)

find_package(catkin REQUIRED)
catkin_package(
)
include_directories(
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_install_python(PROGRAMS nodes/nmea_publisher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)