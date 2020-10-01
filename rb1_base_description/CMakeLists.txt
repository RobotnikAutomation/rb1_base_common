cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_description)

find_package(catkin REQUIRED COMPONENTS
  roslaunch
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

install(
  DIRECTORY launch meshes urdf robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
