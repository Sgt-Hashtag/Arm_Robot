#! /usr/bin/python3

from std_srvs.srv import SetBool, SetBoolResponse  # Import the correct response
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, MoveGroupCommander

# Function to pause the execution
def pause_execution(request):
    move_group.stop()  # Stop the robot's movement
    return SetBoolResponse(success=True, message="Execution Paused")

# Function to resume the execution
def resume_execution(request):
    target_pose = move_group.get_current_pose().pose  # Use current pose or set a new one
    move_group.set_pose_target(target_pose)
    move_group.go(wait=True)  # Resume the robot's movement
    return SetBoolResponse(success=True, message="Execution Resumed")

def main():
    rospy.init_node('pause_resume_execution_service')

    # Initialize the MoveGroupCommander for the robot's planning group (e.g., 'arm')
    global move_group
    move_group = MoveGroupCommander("arm_group")

    # Define the custom services for pausing and resuming execution
    pause_service = rospy.Service('/pause_execution', SetBool, pause_execution)
    resume_service = rospy.Service('/resume_execution', SetBool, resume_execution)

    rospy.loginfo("Services are running...")
    rospy.spin()

if __name__ == "__main__":
    main()
