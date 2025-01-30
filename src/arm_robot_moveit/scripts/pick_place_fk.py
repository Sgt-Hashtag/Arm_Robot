#! /usr/bin/python3

import rospy
import sys
import math
import actionlib
import moveit_commander
from moveit_msgs.msg import CollisionObject, ExecuteTrajectoryAction, ExecuteTrajectoryGoal
import geometry_msgs.msg
from shape_msgs.msg import SolidPrimitive
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class ArmRobot:

	def __init__(self, Group_Name, Grasping_Group_Name):
		'''
		Function to Intialize the Node
		'''
		rospy.loginfo("--- Starting initialising a robotics node ---")
    		
		#initialize moveit Commander
		self._commander = moveit_commander.roscpp_initialize(sys.argv)
		
		#RobotCommander Initialize-Outerlevel Interface to Robot
		self._robot = moveit_commander.RobotCommander()
		
		#instantiate planning scene
		self._scene = moveit_commander.PlanningSceneInterface()
		
		#defining move groups
		self._planning_group = Group_Name
		self._grasping_group = Grasping_Group_Name
		
		#Instantiate MoveGroupCommander Object	
		self._group = moveit_commander.MoveGroupCommander(self._planning_group)
		self._eef_group = moveit_commander.MoveGroupCommander(self._grasping_group)
		
		#Get the planning frame, end effector link and the robot group names
		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		self._group.set_goal_position_tolerance(1E-2)
		self._group.set_goal_orientation_tolerance(1E-3)
		self._group.set_planning_time(15) #setting planning time in seconds
	
		self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
		self._execute_trajectory_client.wait_for_server()
		#print the info
		#here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
		#The '\033[0m' is added at the end of string to reset the terminal colours to default
		rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
		rospy.loginfo('\033[95m' + " >>> ArmRobot initialization is done." + '\033[0m')
		
		
	def wait_for_state_update(self,obj_id,obj_is_known=False, obj_is_attached=False, timeout=2):
		scene = self._scene
		start_time = rospy.get_time()
		secondary_time = rospy.get_time()
				
		while(secondary_time-start_time < timeout) and not rospy.is_shutdown():
			attached_objects = scene.get_attached_objects([obj_id])
			is_attached = len(attached_objects.keys()) > 0
			
			#test
			is_known = obj_id in scene.get_known_object_names()
			
			if (obj_is_attached == is_attached) and (obj_is_known == is_known):
				return True
				
			#sleep to promote other threads
			rospy.sleep(0.1)
			secondary_time = rospy.get_time()
			
		#if while loop exited without returning
		return False
	
	def create_object(self,obj_id,pose,ref_frame, dims, timeout=2):

		object = CollisionObject() #object variable type

		object.id = obj_id
		object.header.frame_id = ref_frame

		solid = SolidPrimitive()
		solid.type = solid.BOX
		solid.dimensions = dims
		object.primitives = [solid]
		scene = self._scene

		## First, create an message object of type pose to deifne box positions
		object_pose = geometry_msgs.msg.Pose()
		#box_pose.header.frame_id=self._armrobot.get_planning_frame()
		#object_pose.header.frame_id = ref_frame

		#set the position massage arguments
		object_pose.position.x = pose[0]
		object_pose.position.y = pose[1]
		object_pose.position.z = pose[2]
		
		#creating quaternion from (rpy)
		orientation_ = quaternion_from_euler(pose[3], pose[4], pose[5])
		
		object_pose.orientation.x = orientation_[0]
		object_pose.orientation.y = orientation_[1]
		object_pose.orientation.x = orientation_[2]
		object_pose.orientation.w = orientation_[3]

		#Adding Object 
		object.primitive_poses = [object_pose] 
		object.operation = object.ADD
		#Add the box in the scene
		scene.add_object(object)
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
		#Check if the box is added sucessfully
		return self.wait_for_state_update(obj_id, obj_is_known=True, timeout=timeout)
	
	def gripper_action(self,gripper, action="open"):
		'''
		Function to execute the required action of the gripper.

		Args:
			gripper:    group_name of gripper in the manipulator

			action:         action to perform. Allowed values ["open", "close"]

		Returns:
			None.
		'''
		print("Executing gripper action = ", action)

		if(action == "open"):
			self.set_pose(gripper,"gripper_open")
		elif(action == "close"):
			self.set_pose(gripper,"gripper_close")
		else:
			print("Action undefined") 

	def set_pose(self,group,arg_pose_name):
		group.set_named_target(arg_pose_name)
		plan_success, plan, planning_time, error_code = group.plan()
		goal=ExecuteTrajectoryGoal()
		goal.trajectory = plan
		self._execute_trajectory_client.send_goal(goal)
		self._execute_trajectory_client.wait_for_result()
		rospy.loginfo('\033[32m' + "Now at: {}".format(arg_pose_name) + '\033[0m')

	def pick_action(self,scene, robot, arm, gripper,box):
		# Gripper open
		self.gripper_action(gripper, action="open")
		# Sleep
		rospy.sleep(0.5)

		# 1) Move to pre-pick location
		self.set_pose(arm,"pre_pick_pose_1")
		# Sleep
		rospy.sleep(0.5)

		# 2) Move to pick location
		self.set_pose(arm,"pick_pose_1")
		# Sleep
		rospy.sleep(0.5)

		# 3) Close the gripper and attach the object
		# Close gripper
		self.gripper_action(gripper, action="close")
		# Attaching the object
		eef_frame = arm.get_end_effector_link()
		grasping_group = self._grasping_group
		touch_links= robot.get_link_names(group=grasping_group)
		scene.attach_box(eef_frame, box, touch_links=touch_links)
		
		# Sleep
		rospy.sleep(0.5)

		# 4) Move to post-pick location
		# Define post-pick pose
		post_pick_pose = "pre_pick_pose_1"
		# Move
		self.set_pose(arm, post_pick_pose)
		# Sleep
		rospy.sleep(0.5)
		return None        
	
	# Object Place function
	def place_action(self, scene, arm, gripper, box):
		
		# 1) Move to pre-drop location
		# Move
		pre_place_pose = "pre_place_pose"
		self.set_pose(arm, pre_place_pose)
		# Sleep
		rospy.sleep(0.5)

		# 2) Move to drop location
		place_pose = "place_pose"
		self.set_pose(arm, place_pose)
		# Sleep
		rospy.sleep(0.5)

		# 3) Open the gripper and detach the object
		# Open gripper
		self.gripper_action(gripper, action="open")
		# Detaching the object
		eef_frame = arm.get_end_effector_link()
		scene.remove_attached_object(eef_frame, name=box)
		# Sleep
		rospy.sleep(0.5)

		# 4) Move to post-drop location
		# Define post-drop pose
		post_place_pose="pre_place_pose"
		# Move
		self.set_pose(arm, post_place_pose)
		# Sleep
		rospy.sleep(0.5)
		return None

	#==================================================================
	def pick_action_reverse(self,scene, robot, arm, gripper,box):
		# Gripper open
		self.gripper_action(gripper, action="open")
		# Sleep
		rospy.sleep(0.5)

		# 1) Move to pre-pick location
		self.set_pose(arm,"pre_place_pose")
		# Sleep
		rospy.sleep(0.5)

		# 2) Move to pick location
		self.set_pose(arm,"place_pose")
		# Sleep
		rospy.sleep(0.5)

		# 3) Close the gripper and attach the object
		# Close gripper
		self.gripper_action(gripper, action="close")
		# Attaching the object
		eef_frame = arm.get_end_effector_link()
		grasping_group = self._grasping_group
		touch_links= robot.get_link_names(group=grasping_group)
		scene.attach_box(eef_frame, box, touch_links=touch_links)
		
		# Sleep
		rospy.sleep(0.5)

		# 4) Move to post-pick location
		# Define post-pick pose
		post_pick_pose = "pre_place_pose"
		# Move
		self.set_pose(arm, post_pick_pose)
		# Sleep
		rospy.sleep(0.5)
		return None        
	
	# Object Place function
	def place_action_reverse(self, scene, arm, gripper, box):
		
		# 1) Move to pre-drop location
		# Move
		pre_place_pose = "pre_pick_pose_1"
		self.set_pose(arm, pre_place_pose)
		# Sleep
		rospy.sleep(0.5)

		# 2) Move to drop location
		place_pose = "pick_pose_1"
		self.set_pose(arm, place_pose)
		# Sleep
		rospy.sleep(0.5)

		# 3) Open the gripper and detach the object
		# Open gripper
		self.gripper_action(gripper, action="open")
		# Detaching the object
		eef_frame = arm.get_end_effector_link()
		scene.remove_attached_object(eef_frame, name=box)
		# Sleep
		rospy.sleep(0.5)

		# 4) Move to post-drop location
		# Define post-drop pose
		post_place_pose="pre_pick_pose_1"
		# Move
		self.set_pose(arm, post_place_pose)
		# Sleep
		rospy.sleep(0.5)
		return None
	#==================================================================

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class ArmRobot Deleted." + '\033[0m')

def main():
	#Initialize a ROS Node
	rospy.init_node('add_attach_detach_objects_in_Rviz', anonymous=True)

	robotArm = ArmRobot("arm_group", "gripper_group")
	robotArm._scene.remove_world_object()
	rospy.loginfo("-- Adding Objects --")
	# robotArm.create_object(obj_id="ceiling", ref_frame=robotArm._planning_frame,pose=[0.0,0,0.30,0.0,0.0,0.0],dims=[0.4,0.4,0.01])
	robotArm.create_object(obj_id="table1", ref_frame=robotArm._planning_frame,pose=[0.2,0,-0.01,0.0,0.0,0.0],dims=[0.2,0.5,0.01])
	robotArm.create_object(obj_id="table2", ref_frame=robotArm._planning_frame,pose=[-0.2,0,-0.01,0.0,0.0,0.0],dims=[0.2,0.5,0.01])
	robotArm.create_object(obj_id="box1", ref_frame=robotArm._planning_frame,pose=[0.2635,-0.0275,0.0300,0.0,0.0,0.0],dims=[0.04,0.04,0.04])
	#robotArm.add_box("package$2",0.00,0.6318,0.015,0.03,0.03,0.03)
	rospy.loginfo("Picking object 1")

	#robotArm.pick_action(robotArm._scene, robotArm._robot, robotArm._group, robotArm._eef_group,box="box1")
	# robotArm.set_pose(robotArm._group,"arm_home")
	# Place box1
	#rospy.loginfo("Placing object 1")
	#robotArm.place_action(robotArm._scene, robotArm._group, robotArm._eef_group, box="box1")

	#==================================================================
	for i in range(5):
		robotArm.pick_action(robotArm._scene, robotArm._robot, robotArm._group, robotArm._eef_group,box="box1")
		rospy.loginfo("Placing object 1")
		robotArm.place_action(robotArm._scene, robotArm._group, robotArm._eef_group, box="box1")

		robotArm.pick_action_reverse(robotArm._scene, robotArm._robot, robotArm._group, robotArm._eef_group,box="box1")
		rospy.loginfo("Placing object 1")
		robotArm.place_action_reverse(robotArm._scene, robotArm._group, robotArm._eef_group, box="box1")

	#==================================================================

	robotArm.set_pose(robotArm._group,"arm_home")
	robotArm.set_pose(robotArm._eef_group,"gripper_close")
	robotArm._scene.remove_world_object()
    
if __name__ == '__main__':
    main()      
