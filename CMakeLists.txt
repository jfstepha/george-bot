cmake_minimum_required(VERSION 2.8.3)
project(george)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  std_msgs
  message_generation
  genmsg 
  actionlib_msgs 
  trajectory_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   Appendage_state.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   Home.srv
   Stop.srv
 )

add_action_files(DIRECTORY action FILES FollowJointTrajectory.action)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES actionlib_msgs trajectory_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES george
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime common_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories( include
  ${catkin_INCLUDE_DIRS}
  /usr/share/arduino/x86/cores/arduino
  /usr/share/arduino/x86/variants/galileo_fab_d
  /usr/share/arduino/x86/libraries
  /usr/share/arduino/x86/libraries/SPI
)

## Declare a cpp library
# add_library(george
#   src/${PROJECT_NAME}/george.cpp
# )

## Declare a cpp executable
add_executable(
  foot_sensors 
    src/foot_sensors.cpp 
    /usr/share/arduino/x86/libraries/SPI/SPI.cpp
    /usr/share/arduino/x86/cores/arduino/RingBuffer.cpp
    /usr/share/arduino/x86/cores/arduino/TTYUART.cpp
    /usr/share/arduino/x86/cores/arduino/WString.cpp
    /usr/share/arduino/x86/cores/arduino/interrupt.c
    /usr/share/arduino/x86/cores/arduino/mux.c
    /usr/share/arduino/x86/cores/arduino/sysfs.c
    /usr/share/arduino/x86/variants/galileo_fab_d/variant.cpp
    /usr/share/arduino/x86/cores/arduino/wiring_digital.c
    /usr/share/arduino/x86/cores/arduino/wiring_analog.c
    /usr/share/arduino/x86/cores/arduino/Print.cpp
    /usr/share/arduino/x86/cores/arduino/UtilTime.cpp
    /usr/share/arduino/x86/cores/arduino/fast_gpio_sc.c
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(george_node george_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(foot_sensors ${catkin_LIBRARIES}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
   scripts/home.py
   scripts/stop.py
   scripts/serial_node.py
   scripts/follow_joint_trajectory_head.py
   scripts/follow_joint_trajectory_larm.py
   scripts/follow_joint_trajectory_lleg.py
   scripts/follow_joint_trajectory_rarm.py
   scripts/follow_joint_trajectory_rleg.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
# install(TARGETS george george_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/rqt_george.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY meshes DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_george.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
