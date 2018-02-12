cmake_minimum_required(VERSION 2.8.3)
SET(ICMAKER_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icmaker")
INCLUDE(${ICMAKER_DIRECTORY}/CMakeLists.txt)

project(schunk_gripper_communication)


## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(icl_core REQUIRED)
find_package(icl_hardware_can REQUIRED)
find_package(icl_comm OPTIONAL_COMPONENTS icl_comm_websocket)
#TODO find out why the below isn't working
ADD_SUBDIRECTORY(src/icl_hardware_canopen)

find_package(icl_hardware_canopen REQUIRED)
find_package(PythonLibs REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  controller_manager
  moveit_ros_planning_interface
  hardware_interface
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf
  urdf
  trac_ik_kinematics_plugin
  trac_ik_lib
)

add_definitions( ${icl_hardware_can_DEFINITIONS} )
add_definitions( ${icl_hardware_canopen_DEFINITIONS} )
add_definitions( ${icl_core_DEFINITIONS} )
add_definitions( ${icl_comm_DEFINITIONS} )

if(icl_comm_icl_comm_websocket_FOUND)
  message(STATUS "Building schunk_gripper_communication with icl_comm_websocket support." )
else ()
  message(STATUS "Building schunk_gripper_communication without icl_comm_websocket support." )
endif()
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Num.msg
)


## Generate services in the 'srv' folder
add_service_files(
   FILES
   HomeAll.srv
   HomeWithIDs.srv
   HomeWithJointNames.srv
   schunk_gripper.srv
)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   control_msgs
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
  INCLUDE_DIRS include
  LIBRARIES schunk_gripper_communication
  CATKIN_DEPENDS 
  actionlib
  controller_manager
  hardware_interface
  joint_limits_interface
  std_srvs
  roscpp 
  rospy 
  std_msgs 
  message_runtime
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
#${python-config --includes}
  ${catkin_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIR}
  ${icl_hardware_can_INCLUDE_DIRS}
  ${icl_hardware_canopen_INCLUDE_DIRS}
  ${icl_core_INCLUDE_DIRS}
  ${icl_comm_INCLUDE_DIRS}
)


#add_executable(collision_contact_tutorial
#    src/collision_contact_tutorial.cpp
#    src/interactive_robot.cpp
#    src/imarker.cpp
#    src/pose_string.cpp
#    )
#target_link_libraries(collision_contact_tutorial
#    ${catkin_LIBRARIES}
#    ${Boost_LIBRARIES}
#    interactive_markers
    
#    )


add_executable(schunk_gripper_communication_node
    src/SchunkCanopenNode.cpp
    src/SchunkCanopenHardwareInterface.cpp
    )




add_dependencies(schunk_gripper_communication_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(schunk_gripper_communication_node
    ${icl_hardware_LIBRARIES}
    ${icl_hardware_canopen_LIBRARIES}
    ${icl_core_LIBRARIES}
    ${icl_comm_INCLUDE_DIRS}
    ${catkin_LIBRARIES}
)



add_executable(schunk_gripper_communication_move_home_node
    src/MoveToHomeNode.cpp
    )

add_dependencies(schunk_gripper_communication_move_home_node ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(schunk_gripper_communication_move_home_node
    ${icl_hardware_LIBRARIES}
    ${icl_hardware_canopen_LIBRARIES}
    ${icl_core_LIBRARIES}
    ${icl_comm_INCLUDE_DIRS}
    ${catkin_LIBRARIES}
)


add_executable(schunk_gripper_server 
    src/schunk_gripper_server.cpp
    src/set_schunk.cpp
    )

target_link_libraries(schunk_gripper_server 
    ${catkin_LIBRARIES} 
    ${PYTHON_LIBRARIES})


add_dependencies(schunk_gripper_server 
    schunk_gripper_communication_gencpp)
add_executable(schunk_gripper_client 
    src/schunk_gripper_client.cpp
    src/set_schunk.cpp
    )

target_link_libraries(schunk_gripper_client 
    ${catkin_LIBRARIES} 
    ${PYTHON_LIBRARIES})
add_dependencies(schunk_gripper_client schunk_gripper_communication_gencpp)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/schunk_gripper_communication.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/schunk_gripper_communication_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
## Mark executables and/or libraries for installation
 install(TARGETS schunk_gripper_communication_node schunk_gripper_communication_move_home_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
 )


#config files
install(DIRECTORY etc/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/etc)

#config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)

#Meshes
install(DIRECTORY meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes)

#URDF
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf)

# install the launch files
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "setup_assistant.launch" EXCLUDE)

install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_schunk_gripper_communication.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
