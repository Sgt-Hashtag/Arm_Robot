#! /usr/bin/python3

import rospy
import sys
import math
import actionlib
import moveit_commander
import geometry_msgs.msg
from moveit_commander import MoveGroupCommander

# Initialize the move group and the robot's planning interface
move_group = MoveGroupCommander("arm_group")  # Replace "arm" with your move group name
move_group.set_planning_time(10)

# Define the target position (xyz) and orientation (quaternion or Euler angles)
target_pose = geometry_msgs.msg.Pose()
target_pose.position.x = 0.15  # Example x position
target_pose.position.y = -0.02  # Example y position
target_pose.position.z = 0.24  # Example z position

# Define the rotation (example: approach at 45 degrees)
# Rotation defined as a quaternion (can also use Euler angles)
target_pose.orientation.w = 0.0  # w
target_pose.orientation.x = 0.0    # x
target_pose.orientation.y = 0.0  # y
target_pose.orientation.z = 0.0    # z

# Set the target pose
move_group.set_pose_target(target_pose)

# Plan and execute the motion
plan = move_group.go(wait=True)
