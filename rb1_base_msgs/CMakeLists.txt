cmake_minimum_required(VERSION 2.8.3)
project(rb1_base_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  actionlib_msgs
  actionlib
)



## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Elevator.msg
   ControllerSpeeds.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SetElevator.srv
)

add_action_files(
   DIRECTORY action
   FILES
   SetElevator.action 
)
## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
)


catkin_package(

)

###########
## Build ##
###########


include_directories(
  ${catkin_INCLUDE_DIRS}
)

