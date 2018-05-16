cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_localization)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_filters
  robot_localization
  robotnik_msgs
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  imu_complementary_filter
)

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
#  LIBRARIES rb1_base_localization
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    message_filters
    robot_localization
    robotnik_msgs
    roscpp
    std_msgs
    std_srvs
    tf
    tf2
    tf2_geometry_msgs
    tf2_ros
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(robot_localization_utils src/robot_localization_utils.cpp)
add_dependencies(robot_localization_utils robotnik_msgs_generate_messages_cpp)
target_link_libraries(robot_localization_utils ${catkin_LIBRARIES})
