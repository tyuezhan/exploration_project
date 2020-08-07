cmake_minimum_required(VERSION 2.8.3)
project(ddk_nav_2d)

add_definitions(-std=c++11 -g -O3 -ftree-vectorize -march=native)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
  tf2
  tf2_eigen
  tf2_geometry_msgs
  pluginlib
  actionlib
  actionlib_msgs
  geometry_msgs
  nav2d_navigator
  kr_mav_msgs
  kr_tracker_msgs
  eigen_conversions
  visualization_msgs
  kr_replanning_msgs
)

find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(PCL 1.7 REQUIRED)


################################################
## Declare ROS messages, services and actions ##
################################################

## Generate actions in the 'action' folder
add_action_files(
  FILES
  Explore.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  actionlib_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES RobotNavigator MapInflationTool
  CATKIN_DEPENDS
    roscpp
    message_runtime
    tf2
    tf2_eigen
    tf2_geometry_msgs
    actionlib
    actionlib_msgs
    geometry_msgs
    nav2d_navigator
    kr_mav_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${CHOLMOD_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

## Declare a cpp library
add_library(MapInflationTool src/MapInflationTool.cpp)
add_library(nav2d src/nav2d.cpp)
add_library(frontier_planner src/frontier_planner.cpp)

## Declare a cpp executable
add_executable(navigator src/navigator.cpp)
add_executable(explore_client src/explore_client.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(navigator ddk_nav_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(explore_client ddk_nav_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(nav2d ddk_nav_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
add_dependencies(frontier_planner ddk_nav_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(explore_client ${catkin_LIBRARIES})
target_link_libraries(navigator nav2d ${catkin_LIBRARIES})
target_link_libraries(nav2d MapInflationTool ${catkin_LIBRARIES})
target_link_libraries(nav2d ${CHOLMOD_LIBRARY} nlopt)
target_link_libraries(nav2d frontier_planner)
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(
  TARGETS
    navigator
    explore_client
    nav2d
    MapInflationTool
    frontier_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nav2d_navigator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
