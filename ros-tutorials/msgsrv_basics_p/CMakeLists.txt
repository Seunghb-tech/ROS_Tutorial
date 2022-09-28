cmake_minimum_required(VERSION 3.0.2)
project(msgsrv_basics_p)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   RobotStatus.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   WarningSwitch.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES msgsrv_basics_p
#  CATKIN_DEPENDS geometry_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
