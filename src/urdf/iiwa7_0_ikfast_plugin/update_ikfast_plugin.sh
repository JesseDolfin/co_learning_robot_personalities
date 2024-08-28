search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=iiwa7.srdf
robot_name_in_srdf=iiwa7
moveit_config_pkg=iiwa7_moveit_config
robot_name=iiwa7
planning_group_name=0
ikfast_plugin_pkg=iiwa7_0_ikfast_plugin
base_link_name=iiwa_link_0
eef_link_name=iiwa_link_ee
ikfast_output_path=/home/jesse/Thesis/co_learning_robot_personalities/src/urdf/iiwa7_0_ikfast_plugin/src/iiwa7_0_ikfast_solver.cpp
eef_direction="0 0 1"

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  --eef_direction $eef_direction\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
