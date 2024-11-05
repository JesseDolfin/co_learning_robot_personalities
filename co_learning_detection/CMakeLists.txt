cmake_minimum_required(VERSION 3.0.2)
project(co_learning_detection)

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
  src/co_learning_detection/detection.py
  src/co_learning_detection/object_detection_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install any necessary files (if you have launch files or other resources)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

