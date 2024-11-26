cmake_minimum_required(VERSION 3.0.2)
project(co_learning_controllers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
  co_learning_messages
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    actionlib_msgs
    co_learning_messages
)

## Install Python modules
catkin_python_setup()

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Install scripts
catkin_install_python(PROGRAMS
  src/co_learning_controllers/control_node.py
  src/co_learning_controllers/hand_controller.py
  src/co_learning_controllers/robot_controller.py
  src/co_learning_controllers/questionaire_controller.py
  src/co_learning_controllers/handover_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)