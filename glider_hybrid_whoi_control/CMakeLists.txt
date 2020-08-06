cmake_minimum_required(VERSION 2.8.3)
project(glider_hybrid_whoi_control)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
