#!/usr/bin/env python3

import rospy
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory():
    # Initialize the ROS node
    rospy.init_node('robot_arm_trajectory_client')

    # Create an action client
    client = actionlib.SimpleActionClient('arm_robot/robot_arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    
    # Wait for the action server to start
    client.wait_for_server()
    rospy.loginfo("Connected to robot_arm_controller/follow_joint_trajectory")

    # Create a trajectory goal
    goal = FollowJointTrajectoryGoal()
    goal.trajectory = JointTrajectory()
    
    # Define the joint names (modify these according to your robot's joint names)
    goal.trajectory.joint_names = ['joint1', 'joint2', 'joint3']
    
    # Create trajectory points
    point = JointTrajectoryPoint()
    point.positions = [1.0, 0.5, -0.5]  # Target positions for each joint
    point.velocities = [0.0, 0.0, 0.0]  # Target velocities (optional)
    point.time_from_start = rospy.Duration(2.0)  # Time to reach this point

    goal.trajectory.points.append(point)

    # Optionally add more points to the trajectory
    # For example, another point with different positions:
    point2 = JointTrajectoryPoint()
    point2.positions = [0.5, -0.5, 0.5]
    point2.velocities = [0.0, 0.0, 0.0]
    point2.time_from_start = rospy.Duration(4.0)

    goal.trajectory.points.append(point2)

    # Send the goal to the action server
    client.send_goal(goal)

    # Wait for the result
    client.wait_for_result()
    rospy.loginfo("Trajectory executed successfully.")

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass

