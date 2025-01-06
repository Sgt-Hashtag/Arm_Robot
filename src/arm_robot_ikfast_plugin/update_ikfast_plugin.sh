search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=arm_robot.srdf
robot_name_in_srdf=arm_robot
moveit_config_pkg=arm_robot_moveit_config
robot_name=arm_robot
planning_group_name=arm_robot
ikfast_plugin_pkg=arm_robot_ikfast_plugin
base_link_name=base_link
eef_link_name=gripper_center
ikfast_output_path=/home/isaac/new_catkin_ws/arm_robot_ikfast_plugin/src/arm_robot_ikfast_solver.cpp
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
