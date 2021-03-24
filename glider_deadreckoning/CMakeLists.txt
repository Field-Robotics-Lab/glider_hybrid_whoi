cmake_minimum_required(VERSION 3.0.2)
project(glider_deadreckoning)

find_package(catkin REQUIRED)
catkin_package(
)
include_directories(
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

catkin_install_python(PROGRAMS nodes/deadreckoning_estimator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)