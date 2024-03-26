# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;control_toolbox;transmission_interface;qb_device_hardware_interface".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lqb_hand_hardware_interface".split(';') if "-lqb_hand_hardware_interface" != "" else []
PROJECT_NAME = "qb_hand_hardware_interface"
PROJECT_SPACE_DIR = "/home/jesse/Thesis/code_jesse/qb_hand/install"
PROJECT_VERSION = "3.0.3"
