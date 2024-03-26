# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "qb_device_srvs: 0 messages, 6 services")

set(MSG_I_FLAGS "-Iqb_device_msgs:/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(qb_device_srvs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" NAME_WE)
add_custom_target(_qb_device_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_device_srvs" "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" ""
)

get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" NAME_WE)
add_custom_target(_qb_device_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_device_srvs" "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" "qb_device_msgs/Info"
)

get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" NAME_WE)
add_custom_target(_qb_device_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_device_srvs" "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" ""
)

get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" NAME_WE)
add_custom_target(_qb_device_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_device_srvs" "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" ""
)

get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" NAME_WE)
add_custom_target(_qb_device_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_device_srvs" "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" ""
)

get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" NAME_WE)
add_custom_target(_qb_device_srvs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "qb_device_srvs" "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_cpp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv"
  "${MSG_I_FLAGS}"
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_msgs/msg/Info.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_cpp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_cpp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_cpp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_cpp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
)

### Generating Module File
_generate_module_cpp(qb_device_srvs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(qb_device_srvs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(qb_device_srvs_generate_messages qb_device_srvs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_cpp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_cpp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_cpp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_cpp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_cpp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_cpp _qb_device_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_device_srvs_gencpp)
add_dependencies(qb_device_srvs_gencpp qb_device_srvs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_device_srvs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_eus(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv"
  "${MSG_I_FLAGS}"
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_msgs/msg/Info.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_eus(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_eus(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_eus(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_eus(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
)

### Generating Module File
_generate_module_eus(qb_device_srvs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(qb_device_srvs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(qb_device_srvs_generate_messages qb_device_srvs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_eus _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_eus _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_eus _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_eus _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_eus _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_eus _qb_device_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_device_srvs_geneus)
add_dependencies(qb_device_srvs_geneus qb_device_srvs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_device_srvs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_lisp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv"
  "${MSG_I_FLAGS}"
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_msgs/msg/Info.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_lisp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_lisp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_lisp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_lisp(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
)

### Generating Module File
_generate_module_lisp(qb_device_srvs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(qb_device_srvs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(qb_device_srvs_generate_messages qb_device_srvs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_lisp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_lisp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_lisp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_lisp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_lisp _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_lisp _qb_device_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_device_srvs_genlisp)
add_dependencies(qb_device_srvs_genlisp qb_device_srvs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_device_srvs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_nodejs(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv"
  "${MSG_I_FLAGS}"
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_msgs/msg/Info.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_nodejs(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_nodejs(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_nodejs(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_nodejs(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
)

### Generating Module File
_generate_module_nodejs(qb_device_srvs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(qb_device_srvs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(qb_device_srvs_generate_messages qb_device_srvs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_nodejs _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_nodejs _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_nodejs _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_nodejs _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_nodejs _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_nodejs _qb_device_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_device_srvs_gennodejs)
add_dependencies(qb_device_srvs_gennodejs qb_device_srvs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_device_srvs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_py(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv"
  "${MSG_I_FLAGS}"
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_msgs/msg/Info.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_py(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_py(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_py(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
)
_generate_srv_py(qb_device_srvs
  "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
)

### Generating Module File
_generate_module_py(qb_device_srvs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(qb_device_srvs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(qb_device_srvs_generate_messages qb_device_srvs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/GetMeasurements.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_py _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/InitializeDevice.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_py _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetCommands.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_py _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetPID.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_py _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/Trigger.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_py _qb_device_srvs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jesse/Thesis/code_jesse/qb_hand/src/qbdevice-ros/qb_device_srvs/srv/SetControlMode.srv" NAME_WE)
add_dependencies(qb_device_srvs_generate_messages_py _qb_device_srvs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(qb_device_srvs_genpy)
add_dependencies(qb_device_srvs_genpy qb_device_srvs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS qb_device_srvs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/qb_device_srvs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_cpp)
  add_dependencies(qb_device_srvs_generate_messages_cpp std_srvs_generate_messages_cpp)
endif()
if(TARGET qb_device_msgs_generate_messages_cpp)
  add_dependencies(qb_device_srvs_generate_messages_cpp qb_device_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/qb_device_srvs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_eus)
  add_dependencies(qb_device_srvs_generate_messages_eus std_srvs_generate_messages_eus)
endif()
if(TARGET qb_device_msgs_generate_messages_eus)
  add_dependencies(qb_device_srvs_generate_messages_eus qb_device_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/qb_device_srvs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_lisp)
  add_dependencies(qb_device_srvs_generate_messages_lisp std_srvs_generate_messages_lisp)
endif()
if(TARGET qb_device_msgs_generate_messages_lisp)
  add_dependencies(qb_device_srvs_generate_messages_lisp qb_device_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/qb_device_srvs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_nodejs)
  add_dependencies(qb_device_srvs_generate_messages_nodejs std_srvs_generate_messages_nodejs)
endif()
if(TARGET qb_device_msgs_generate_messages_nodejs)
  add_dependencies(qb_device_srvs_generate_messages_nodejs qb_device_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/qb_device_srvs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_srvs_generate_messages_py)
  add_dependencies(qb_device_srvs_generate_messages_py std_srvs_generate_messages_py)
endif()
if(TARGET qb_device_msgs_generate_messages_py)
  add_dependencies(qb_device_srvs_generate_messages_py qb_device_msgs_generate_messages_py)
endif()
