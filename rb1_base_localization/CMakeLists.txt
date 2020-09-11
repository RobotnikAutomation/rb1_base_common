cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_localization)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  message_filters
  robot_localization
  roscpp
  std_msgs
  std_srvs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
  imu_complementary_filter
  amcl
  map_server
  gmapping
)


catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    mavros_msgs
    message_filters
    robot_localization
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
install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY maps
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/amcl_resample.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
