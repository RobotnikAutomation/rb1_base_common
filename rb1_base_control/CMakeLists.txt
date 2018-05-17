cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_control)

find_package(catkin REQUIRED COMPONENTS
   twist_mux
   controller_manager
   )

catkin_package(
)

install(
  DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
