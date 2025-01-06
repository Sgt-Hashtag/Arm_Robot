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
		self._group.set_planning_time(10) #setting planning time in seconds
		self._group.set_planner_id("RRTConnectkConfigDefault")
		
		self.box_name = ""
		self._execute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', ExecuteTrajectoryAction)
		self._execute_trajectory_client.wait_for_server()
		#print the info
		#here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
		#The '\033[0m' is added at the end of string to reset the terminal colours to default
		rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
		rospy.loginfo('\033[95m' + " >>> ArmRobot initialization is done." + '\033[0m')
		
		
	def wait_for_state_update(self,obj_id,obj_is_known=False, obj_is_attached=False, timeout=4):
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
	
	def create_object(self,obj_id,pose,ref_frame, dims, timeout=4):

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
		
		
	# def attach_box(self,name, timeout=4):
	# 	box_name = name
	# 	robot = self._robot
	# 	scene = self._scene
	# 	eef_link = self._eef_link

	# 	grasping_group = self._grasping_group
	# 	touch_links = robot.get_link_names(group=grasping_group)
	# 	rospy.loginfo('\033[95m' + "Touch Link: {}".format(touch_links) + '\033[0m')
	# 	scene.attach_box(eef_link, box_name, touch_links=touch_links)
	# 	#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
	# 	#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

	# 	# We wait for the planning scene to update.
	# 	return self.wait_for_state_update(name,obj_is_known=False, obj_is_attached=True,  timeout=timeout)

	# def detach_box(self,name, timeout=4):
	# 	box_name = name
	# 	scene = self._scene
	# 	eef_link = self._eef_link

	# 	scene.remove_attached_object(eef_link, name=box_name)
	# 	#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
	# 	#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

	# 	return self.wait_for_state_update(name, box_is_known=True, box_is_attached=False, timeout=timeout)
        
	# def remove_box(self,name, timeout=4):
	# 	box_name = name
	# 	scene = self._scene
	# 	scene.remove_world_object(box_name)
	# 	#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
	# 	#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
	# 	return self.wait_for_state_update(name, box_is_known=False, box_is_attached=False,  timeout=timeout)
	

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
			self.set_pose(gripper,"hand_open")
		elif(action == "close"):
			self.set_pose(gripper,"hand_close")
		else:
			print("Action undefined") 
	def set_pose(self,eef,arg_pose_name):
		eef.set_named_target(arg_pose_name)
		plan_success, plan, planning_time, error_code = eef.plan()
		goal=ExecuteTrajectoryGoal()
		goal.trajectory = plan
		self._execute_trajectory_client.send_goal(goal)
		self._execute_trajectory_client.wait_for_result()
		rospy.loginfo('\033[32m' + "Now at: {}".format(arg_pose_name) + '\033[0m')
		
	def move_to_pose(self,arm, goal_pose):
		'''
		Function to move the robot arm to the requested pose

		Args:
			arm:        robot's arm group that should be moved.

			goal_pose:  requested pose (in the planning frame)

		Returns:
			None
		'''
		#NOTE:
		#The planner will create a plan for the robot to reach
		#the goal pose. 

		rospy.logwarn("planning and moving to location")
		gripper_angle = geometry_msgs.msg.Quaternion()

		#Orientation of the tcp(panda_hand_tcp) w.r.t base frame(panda_link0) 
		quaternion = quaternion_from_euler(math.radians(180), 0, math.radians(-45))

		gripper_angle.x = quaternion[0]
		gripper_angle.y = quaternion[1]
		gripper_angle.z = quaternion[2]
		gripper_angle.w = quaternion[3]

		#defining pre-grasp position
		grasp_pose = geometry_msgs.msg.Pose()
		grasp_pose.position.x = goal_pose.position.x
		grasp_pose.position.y = goal_pose.position.y
		grasp_pose.position.z = goal_pose.position.z   
		grasp_pose.orientation = gripper_angle

		print("Exectuting move_to_pose ({} , {},  {})"
				.format(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z))

		arm.set_goal_position_tolerance(0.001) #setting tolerence for pose 
		arm.set_pose_target(grasp_pose) #setting the pose of the end-effector

		plan = arm.go(wait=True) #move the arm to the grasp_pose.

		arm.stop() #ensures that there is no residual movement.
		arm.clear_pose_targets() #to clear the existing targets.

	def pick_action(self,scene, robot, arm, gripper, pick_pose, box):
		# Gripper open
		self.gripper_action(gripper, action="open")
		# Sleep
		rospy.sleep(1)

		# 1) Move to pre-pick location
		# Define pre-pick pose
		pre_pick_pose = geometry_msgs.msg.Pose()
		pre_pick_pose.position.x = pick_pose.position.x
		pre_pick_pose.position.y = pick_pose.position.y
		pre_pick_pose.position.z = pick_pose.position.z + 0.1 # Placing it above the object
		# Move
		self.move_to_pose(arm, pre_pick_pose)
		# Sleep
		rospy.sleep(1)

		# 2) Move to pick location
		self.move_to_pose(arm, pick_pose)
		# Sleep
		rospy.sleep(1)

		# 3) Close the gripper and attach the object
		# Close gripper
		self.gripper_action(gripper, action="close")
		# Attaching the object
		eef_frame = arm.get_end_effector_link()
		grasping_group = self._grasping_group
		touch_links= robot.get_link_names(group=grasping_group)
		scene.attach_box(eef_frame, box, touch_links=touch_links)
		
		# Sleep
		rospy.sleep(1)

		# 4) Move to post-pick location
		# Define post-pick pose
		post_pick_pose = geometry_msgs.msg.Pose()
		post_pick_pose.position.x = pick_pose.position.x
		post_pick_pose.position.y = pick_pose.position.y
		post_pick_pose.position.z = pick_pose.position.z + 0.1
		# Move
		self.move_to_pose(arm, post_pick_pose)
		# Sleep
		rospy.sleep(1)
		return None        
	
	# Object Place function
	def place_action(self, scene, arm, gripper, place_pose, box):
		# 1) Move to pre-drop location
		# Define pre-drop pose
		pre_drop_pose = geometry_msgs.msg.Pose()
		pre_drop_pose.position.x = place_pose.position.x
		pre_drop_pose.position.y = place_pose.position.y
		pre_drop_pose.position.z = place_pose.position.z + 0.001
		# Move
		self.move_to_pose(arm, pre_drop_pose)
		# Sleep
		rospy.sleep(1)

		# 2) Move to drop location
		self.move_to_pose(arm, place_pose)
		# Sleep
		rospy.sleep(1)

		# 3) Open the gripper and detach the object
		# Open gripper
		self.gripper_action(gripper, action="open")
		# Detaching the object
		eef_frame = arm.get_end_effector_link()
		scene.remove_attached_object(eef_frame, name=box)
		# Sleep
		rospy.sleep(1)

		# 4) Move to post-drop location
		# Define post-drop pose
		post_drop_pose = geometry_msgs.msg.Pose()
		post_drop_pose.position.x = place_pose.position.x
		post_drop_pose.position.y = place_pose.position.y
		post_drop_pose.position.z = place_pose.position.z + 0.1
		# Move
		self.move_to_pose(arm, post_drop_pose)
		# Sleep
		rospy.sleep(1)
		return None

	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class ArmRobot Deleted." + '\033[0m')

def main():
	#Initialize a ROS Node
	rospy.init_node('add_attach_detach_objects_in_Rviz', anonymous=True)

	robotArm = ArmRobot("arm_robot", "hand_ee")
	robotArm._scene.remove_world_object()
	rospy.loginfo("-- Adding Objects --")
	robotArm.create_object(obj_id="table1", ref_frame=robotArm._planning_frame,pose=[0.2,0,-0.01,0.0,0.0,0.0],dims=[0.2,0.5,0.01])
	robotArm.create_object(obj_id="table2", ref_frame=robotArm._planning_frame,pose=[-0.2,0,-0.01,0.0,0.0,0.0],dims=[0.2,0.5,0.01])
	robotArm.create_object(obj_id="box1", ref_frame=robotArm._planning_frame,pose=[0.2,0,0.025,0.0,0.0,0.0],dims=[0.05,0.05,0.05])
	#robotArm.add_box("package$2",0.00,0.6318,0.015,0.03,0.03,0.03)
	rospy.loginfo("Picking object 1")
	pick_pose_1 = geometry_msgs.msg.Pose()
	pick_pose_1.position.x = 0.5
	pick_pose_1.position.y = -0.1
	pick_pose_1.position.z = 0.3+((0.01+0.025+0.02)/2.0)
	robotArm.pick_action(robotArm._scene, robotArm._robot, robotArm._group, robotArm._eef_group, pick_pose_1, box="box1")
	# Place box1
	place_pose_1 = geometry_msgs.msg.Pose()
	place_pose_1.position.x = -0.15
	place_pose_1.position.y = 0.5
	place_pose_1.position.z = 0.3+((0.01+0.025+0.02)/2)
	robotArm.place_action(robotArm._scene, robotArm._group, robotArm._eef_group, place_pose_1, box="box1")
	# while not rospy.is_shutdown():
	# 	task = input("Enter add to add object, attch to attach object, detach to detach object and remove to remove object: ")            
	# 	if task == "attach":
	# 		response = robotArm.attach_box("package$1")
	# 		if response == True:
	# 			rospy.loginfo('\033[32m' + "Attached Object in Rviz: {}".format(robotArm.box_name) + '\033[0m')
	# 	if task == "detach":
	# 		response = robotArm.detach_box("package$1")
	# 		if response == True:
	# 			rospy.loginfo('\033[32m' + "Detached Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
	# 	if task == "remove":
	# 		response = robotArm.remove_box("package$1")
	# 		if response == True:
	# 			rospy.loginfo('\033[32m' + "Removed Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
	# 	if task == "x":
	# 	    break
	#rospy.spin()
	# quit()
	#delete the robotArm object at the end of code
	# del robotArm
	robotArm._scene.remove_world_object()
    
if __name__ == '__main__':
    main()      
