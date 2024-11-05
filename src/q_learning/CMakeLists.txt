cmake_minimum_required(VERSION 3.0.2)
project(q_learning)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  co_learning_messages
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    co_learning_messages
)

## Install Python modules
catkin_python_setup()

## Install scripts
catkin_install_python(PROGRAMS
  src/q_learning/QLearnAgent.py
  src/q_learning/CoLearnEnvironment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
