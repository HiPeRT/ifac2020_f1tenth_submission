cmake_minimum_required(VERSION 2.8.3)
project(muretto)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  sensor_msgs
  tf
  ackermann_msgs
  obstacle_detector
  message_generation
  visualization_msgs
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_message_files(
  FILES
  Strategy.msg
)

generate_messages(DEPENDENCIES std_msgs)

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
#  LIBRARIES velocities
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp tf ackermann_msgs sensor_msgs std_msgs obstacle_detector message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

set(LINK_LIBRARIES
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(${PROJECT_NAME}_node src/muretto_node.cpp include/occupancy_grid.h src/occupancy_grid.cpp)
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  yaml-cpp
)