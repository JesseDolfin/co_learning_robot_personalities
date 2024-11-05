cmake_minimum_required(VERSION 3.0.2)
project(co_learning_secondary_task)

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
  src/co_learning_secondary_task/secondary_task.py
  src/co_learning_secondary_task/pantograph.py
  src/co_learning_secondary_task/pshape.py
  src/co_learning_secondary_task/pyhapi.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install resources
install(DIRECTORY src/co_learning_secondary_task/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.mp3" PATTERN "*.png"
)
