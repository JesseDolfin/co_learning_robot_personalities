cmake_minimum_required(VERSION 3.0.2)
project(secondary_task_messages)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Secondary_task.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )


## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
