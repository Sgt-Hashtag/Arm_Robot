#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import tf
from geometry_msgs.msg import Pose

def get_end_effector_pose():
    # Initialize moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_end_effector_pose', anonymous=True)

    # Initialize the robot, scene, and planning group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group = moveit_commander.MoveGroupCommander("arm_group")  # Replace "arm" with your robot's planning group

    # Get the current pose of the end effector
    end_effector_pose = group.get_current_pose().pose

    # Extract quaternion from pose
    quaternion = (
        end_effector_pose.orientation.x,
        end_effector_pose.orientation.y,
        end_effector_pose.orientation.z,
        end_effector_pose.orientation.w
    )

    # Convert quaternion to Euler angles (roll, pitch, yaw)
    euler = tf.transformations.euler_from_quaternion(quaternion)

    # Extract roll, pitch, yaw values
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    # Print out the result
    rospy.loginfo("End Effector Pose: \nPosition: {}\nOrientation (Euler Angles): Roll: {}, Pitch: {}, Yaw: {}".format(
        end_effector_pose.position, roll, pitch, yaw))

    # Close moveit_commander
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    get_end_effector_pose()

