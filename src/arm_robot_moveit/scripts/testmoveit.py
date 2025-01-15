#! /usr/bin/python3

import rospy
import sys
import math
import actionlib
import moveit_commander
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander
from tf.transformations import quaternion_from_euler

# Initialize the move group and the robot's planning interface
move_group = MoveGroupCommander("arm_group")  # Replace "arm" with your move group name
move_group.set_planning_time(10)


# PREPICK
# target_pose = geometry_msgs.msg.Pose()
# target_pose.position.x =  0.2275# Example x position 
# target_pose.position.y = -0.0235 # Example y position
# target_pose.position.z = 0.2300 # Example z position
# #angle = 45 # 45 degrees
# angle = [-90.12,53.82,-6.046]
# #radians = angle * 3.14159 / 180.0
# # quaternion = quaternion_from_euler(angle[0]* 3.14159 / 180.0, angle[1]* 3.14159 / 180.0 ,angle[2]* 3.14159 / 180.0 )
# quaternion = quaternion_from_euler(-1.5716,0.0, -0.1044)
# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]


# # PICK
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.2635 # Example x position 
target_pose.position.y = -0.0275 # Example y position
target_pose.position.z = 0.0300 # Example z position
#radians = angle * 3.14159 / 180.0  
quaternion = quaternion_from_euler(-1.5716,0.7727, -0.1050)
target_pose.orientation.x = quaternion[0]
target_pose.orientation.y = quaternion[1]
target_pose.orientation.z = quaternion[2]
target_pose.orientation.w = quaternion[3]



# #PRE PLACE
# target_pose = geometry_msgs.msg.Pose()
# target_pose.position.x = 0.27741959403441474  # Example x position 
# target_pose.position.y = -0.029035107795378486 # Example y position
# target_pose.position.z = 0.03188156286045721  # Example z position
# #angle = 45 # 45 degrees
# #radians = angle * 3.14159 / 180.0  
# quaternion = quaternion_from_euler(-1.569974443091994, -0.029394324419701502, 3.0375673556870706)
# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]

# #PLACE
# target_pose = geometry_msgs.msg.Pose()
# target_pose.position.x = -0.2160282624421584 # Example x position 
# target_pose.position.y = 0.022602144807172313 # Example y position
# target_pose.position.z = 0.027039978916076235 # Example z position
# #angle = 45 # 45 degrees
# #radians = angle * 3.14159 / 180.0  
# quaternion = quaternion_from_euler(-1.5692461045024122, 1.045934545633635, 3.03881973044592)
# target_pose.orientation.x = quaternion[0]
# target_pose.orientation.y = quaternion[1]
# target_pose.orientation.z = quaternion[2]
# target_pose.orientation.w = quaternion[3]

# Set the target pose
move_group.set_pose_target(target_pose)

# Plan and execute the motion
plan = move_group.go(wait=True)
