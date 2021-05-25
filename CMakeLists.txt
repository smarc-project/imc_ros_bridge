cmake_minimum_required(VERSION 2.8.3)
project(imc_ros_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp message_runtime message_generation tf2 tf2_geometry_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
#add_subdirectory(external)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

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

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  PlanControl.msg
  PlanControlState.msg
  VehicleState.msg
  EstimatedState.msg
  RemoteState.msg
  SonarData.msg
  DesiredHeading.msg
  DesiredHeadingRate.msg
  DesiredPitch.msg
  DesiredRoll.msg
  DesiredSpeed.msg
  DesiredZ.msg
  PlanDB.msg
  PlanSpecification.msg
  PlanDBInformation.msg
  PlanDBState.msg
  PlanManeuver.msg
  Maneuver.msg
  PolygonVertex.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
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
#  INCLUDE_DIRS include
#  LIBRARIES imc_ros_bridge
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

# THINGS TO DO WHEN ADDING NEW MESSAGES (0/3)

# 1/3
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  external/imc-cxx
  external/md5
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library( md5 external/md5/md5.cpp )
add_library( ros_to_imc_heartbeat src/ros_to_imc/Heartbeat.cpp )
add_library( imc_to_ros_heartbeat src/imc_to_ros/Heartbeat.cpp )
add_library( ros_to_imc_gpsfix src/ros_to_imc/GpsFix.cpp )
add_library( ros_to_imc_goto src/ros_to_imc/Goto.cpp )
add_library( imc_to_ros_goto src/imc_to_ros/Goto.cpp )
add_library( imc_factory external/imc-cxx/IMC/Base/Factory.cpp )
add_library( udp_link src/udp_link.cpp )
add_library( imc_handle src/imc_handle.cpp )
add_library( imc_to_ros_abort src/imc_to_ros/Abort.cpp )
add_library( imc_to_ros_plandb src/imc_to_ros/PlanDB.cpp )
add_library( ros_to_imc_plandb src/ros_to_imc/PlanDB.cpp )
add_library( ros_to_imc_gpsnavdata src/ros_to_imc/GpsNavData.cpp )
add_library( imc_to_ros_plancontrol src/imc_to_ros/PlanControl.cpp )
add_library( ros_to_imc_plancontrolstate src/ros_to_imc/PlanControlState.cpp )
add_library( ros_to_imc_estimatedstate src/ros_to_imc/EstimatedState.cpp )
add_library( ros_to_imc_vehiclestate src/ros_to_imc/VehicleState.cpp )
add_library( ros_to_imc_remotestate src/ros_to_imc/RemoteState.cpp )
add_library( ros_to_imc_sonardata src/ros_to_imc/SonarData.cpp )
add_library( ros_to_imc_DesiredHeading src/ros_to_imc/DesiredHeading.cpp )
add_library( ros_to_imc_DesiredHeadingRate src/ros_to_imc/DesiredHeadingRate.cpp )
add_library( ros_to_imc_DesiredPitch src/ros_to_imc/DesiredPitch.cpp )
add_library( ros_to_imc_DesiredRoll src/ros_to_imc/DesiredRoll.cpp )
add_library( ros_to_imc_DesiredSpeed src/ros_to_imc/DesiredSpeed.cpp )
add_library( ros_to_imc_DesiredZ src/ros_to_imc/DesiredZ.cpp )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(imc_to_ros_plandb imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_plandb imc_ros_bridge_generate_messages_cpp)
add_dependencies(imc_to_ros_plancontrol imc_ros_bridge_generate_messages_cpp)

add_dependencies(ros_to_imc_plancontrolstate imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_vehiclestate imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_sonardata imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_estimatedstate imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_remotestate imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredHeading imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredHeadingRate imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredPitch imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredRoll imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredSpeed imc_ros_bridge_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredZ imc_ros_bridge_generate_messages_cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/imc_ros_bridge_node.cpp)
add_executable(bridge_node src/bridge_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2/3
## Specify libraries to link a library or executable target against
target_link_libraries(md5 ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_heartbeat ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_goto ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_abort ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_plandb ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_plancontrol ${catkin_LIBRARIES})

target_link_libraries(ros_to_imc_heartbeat ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_gpsfix ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_goto ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_plancontrolstate ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_gpsnavdata ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_estimatedstate ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_vehiclestate ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_remotestate ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_sonardata ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_DesiredHeading ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredHeadingRate ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredPitch ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredRoll ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredSpeed ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredZ ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_plandb ${catkin_LIBRARIES})

target_link_libraries(udp_link
  imc_factory
  ${Boost_LIBRARIES}
)

target_link_libraries(imc_handle
  imc_factory
  udp_link
  ${Boost_LIBRARIES}
)

# 3/3
target_link_libraries(bridge_node
  imc_to_ros_heartbeat
  imc_to_ros_goto
  imc_to_ros_abort
  imc_to_ros_plandb
  imc_to_ros_plancontrol
  ros_to_imc_plandb
  ros_to_imc_heartbeat
  ros_to_imc_gpsfix
  ros_to_imc_goto
  ros_to_imc_plancontrolstate
  ros_to_imc_gpsnavdata
  ros_to_imc_estimatedstate
  ros_to_imc_vehiclestate
  ros_to_imc_remotestate
  ros_to_imc_sonardata
  ros_to_imc_DesiredHeading
  ros_to_imc_DesiredHeadingRate
  ros_to_imc_DesiredPitch
  ros_to_imc_DesiredRoll
  ros_to_imc_DesiredSpeed
  ros_to_imc_DesiredZ
  imc_handle
  md5
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


#############
## Install ##
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
install(TARGETS bridge_node md5 ros_to_imc_heartbeat imc_to_ros_heartbeat ros_to_imc_gpsfix ros_to_imc_goto imc_to_ros_goto imc_factory udp_link imc_handle imc_to_ros_abort imc_to_ros_plandb ros_to_imc_plandb ros_to_imc_gpsnavdata imc_to_ros_plancontrol ros_to_imc_plancontrolstate ros_to_imc_estimatedstate ros_to_imc_vehiclestate ros_to_imc_remotestate ros_to_imc_sonardata ros_to_imc_DesiredHeading ros_to_imc_DesiredHeadingRate ros_to_imc_DesiredPitch ros_to_imc_DesiredRoll ros_to_imc_DesiredSpeed ros_to_imc_DesiredZ
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_imc_ros_bridge.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
