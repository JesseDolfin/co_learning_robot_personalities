cmake_minimum_required(VERSION 3.0.2)
project(co_learning_controllers)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs # controller action server 
  std_msgs #TODO: not sure if its needed to build msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cor_tud_controllers
	CATKIN_DEPENDS cor_tud_msgs
#  DEPENDS system_lib
)

#include_directories(
# include
#  ${catkin_INCLUDE_DIRS}
#)


catkin_python_setup()

catkin_install_python(PROGRAMS
  src/control_node.py
  src/hand_controller.py
  src/robot_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
