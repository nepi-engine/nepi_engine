#[[
Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.

This file is part of nepi-engine
(see https://github.com/nepi-engine).

License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
]]
cmake_minimum_required(VERSION 2.8.3)
project(nepi_sdk)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
add_compile_options(-Wall)

# Following relies (sometimes, e.g., Jetpack-installed OpenCV) on a symlink from /usr/share/OpenCV to
# true OpenCV cmake folder (e.g., usr/lib/aarch64-linux-gnu/cmake/opencv4 on Jetson)
set(OpenCV_DIR /usr/share/OpenCV)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
## Note that using COMPONENTS here ensures that the package environment variables
## are combined into the generic corresponding catkin_<ENV_VARIABLE> stuff, so, for example
## we don't need to explicitly link OpenCV_LIBRARIES, instead just link to catkin_LIBRARIES and
## get OpenCV for free (same holds for catkin_INCLUDE_DIRS, etc.)
find_package(catkin REQUIRED
    COMPONENTS
    message_generation
    nepi_ros_interfaces
    roscpp
    image_transport
    cv_bridge
    OpenCV
    sensor_msgs
    rospy
    std_msgs
    pcl_ros
    tf2_ros
    nav_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

## Use the verbose version of make
#set(CMAKE_VERBOSE_MAKEFILE ON)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)


## Generate added messages and services with any dependencies listed here
generate_messages(
     DEPENDENCIES
     std_msgs  # Or other packages containing msgs
     sensor_msgs
     nepi_ros_interfaces
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS image_transport nepi_ros_interfaces
)


#catkin_install_python(PROGRAMS
#  scripts/nepi_ai_detector_img_pub_node.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

###########
## Build 
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include
   ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/sdk_node.cpp
   src/sdk_interface.cpp
   src/sdk_utils.cpp
   src/save_data_interface.cpp
   src/algorithms/approx_moving_avg.cpp

)


 
#############
## Install 
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )


## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES
#  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)


# Install etc files (startup scripts, factory configs, etc.) to the global (not project) etc destination
#install(DIRECTORY etc/
#  DESTINATION ${CATKIN_GLOBAL_ETC_DESTINATION}
#  USE_SOURCE_PERMISSIONS
#)


set( NEPI_SDK_SHARE_FOLDER 
  # Must hard-coded 
  /opt/nepi/ros/share/nepi_sdk/scripts
)

install(DIRECTORY scripts/
  DESTINATION ${NEPI_SDK_SHARE_FOLDER} 
  FILES_MATCHING PATTERN "*.py"
)

set( NEPI_SDK_FOLDER 
  # Must hard-coded 
  /opt/nepi/ros/lib/python3/dist-packages/nepi_sdk
)

install(DIRECTORY sdk/
  DESTINATION ${NEPI_SDK_FOLDER} 
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY ifs/
  DESTINATION ${NEPI_SDK_FOLDER} 
  FILES_MATCHING PATTERN "*.py"
)


#############
## Testing General ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sdk.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



