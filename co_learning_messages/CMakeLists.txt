cmake_minimum_required(VERSION 3.0.2)
project(co_learning_messages)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

## Declare ROS messages
add_message_files(
  FILES
    Database.msg
    Detection.msg
    hand_pose.msg
    secondary_task_message.msg
)

## Generate added messages
generate_messages(
  DEPENDENCIES
    std_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)
