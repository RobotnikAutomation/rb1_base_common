cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_pad)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  diagnostic_updater
)


add_service_files(DIRECTORY srv 
   FILES
    enable_disable_pad.srv
)


generate_messages(
   DEPENDENCIES
   robotnik_msgs sensor_msgs
 )


catkin_package(

  CATKIN_DEPENDS 
    robotnik_msgs
    roscpp 
    sensor_msgs 
    geometry_msgs 
    message_runtime
    diagnostic_updater

)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS} 
)


add_executable(rb1_base_pad src/rb1_base_pad.cpp)

add_dependencies(rb1_base_pad rb1_base_pad_generate_messages_cpp)

target_link_libraries(rb1_base_pad
   ${catkin_LIBRARIES}
 )

install(
  TARGETS rb1_base_pad
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

