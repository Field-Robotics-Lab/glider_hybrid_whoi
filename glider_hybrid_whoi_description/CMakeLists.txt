cmake_minimum_required(VERSION 2.8.3)
project(glider_hybrid_whoi_description)

find_package(catkin REQUIRED)

catkin_package()

# Install python scripts
catkin_install_python(PROGRAMS scripts/spawn_model
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY mesh robots urdf launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)