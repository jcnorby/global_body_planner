cmake_minimum_required(VERSION 3.0.2)
project(global_body_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Set default cmake build type to release
if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  nav_msgs
  message_generation
  grid_map_core
  grid_map_ros
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BodyPlan.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  nav_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES body_planner
 CATKIN_DEPENDS roscpp std_msgs grid_map_core grid_map_ros sensor_msgs nav_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

# JN Added
set(MY_LIB_SOURCES
  src/global_body_planner.cpp
  src/planning_utils.cpp
  src/graph_class.cpp
  src/planner_class.cpp
  src/rrt.cpp
  src/rrt_connect.cpp
  src/rrt_star_connect.cpp
  src/rviz_interface.cpp
  src/fast_terrain_map.cpp
  src/terrain_map_publisher.cpp)

# Declare a C++ library
add_library(${PROJECT_NAME}
  ${MY_LIB_SOURCES}
)


#############
## Planner ##
#############

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
# With catkin_make all packages are built within a single CMake context
# The recommended prefix ensures that target names across packages don't collide
add_executable(global_body_planner_node src/global_body_planner_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(global_body_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(global_body_planner_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

###############
## Interface ##
###############

add_executable(rviz_interface_node src/rviz_interface_node.cpp)
add_executable(terrain_map_publisher_node src/terrain_map_publisher_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(rviz_interface_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

target_link_libraries(terrain_map_publisher_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test test/test_global_body_planner.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${catkin_LIBRARIES})

