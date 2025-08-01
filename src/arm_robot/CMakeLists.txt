cmake_minimum_required(VERSION 3.1.3)

project(arm_robot)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  urdf
  xacro
)

catkin_package(
CATKIN_DEPENDS 
  message_runtime
  geometry_msgs
  roscpp 
  rospy 
  std_msgs
)

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
