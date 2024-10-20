#! /usr/bin/python3

import rospy
import sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class ArmRobot:

	def __init__(self, Group_Name, Grasping_Group_Name):
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
		
		#Get the planning frame, end effector link and the robot group names
		self._planning_frame = self._group.get_planning_frame()
		self._eef_link = self._group.get_end_effector_link()
		self._group_names = self._robot.get_group_names()
		
		self.box_name = ""

		#print the info
		#here the '\033[95m' represents the standard colour "LightMagenta" in terminals. For details, refer: https://pkg.go.dev/github.com/whitedevops/colors
		#The '\033[0m' is added at the end of string to reset the terminal colours to default
		rospy.loginfo('\033[95m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
		rospy.loginfo('\033[95m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
		rospy.loginfo('\033[95m' + "Group Names: {}".format(self._group_names) + '\033[0m')
		rospy.loginfo('\033[95m' + " >>> ArmRobot initialization is done." + '\033[0m')
		
		
	def wait_for_state_update(self,name,box_is_known=False, box_is_attached=False, timeout=4):
		box_name=name
		scene = self._scene
		start_time = rospy.get_time()
		secondary_time = rospy.get_time()
				
		while(secondary_time-start_time < timeout) and not rospy.is_shutdown():
			attached_objects = scene.get_attached_objects([box_name])
			is_attached = len(attached_objects.keys()) > 0
			
			#test
			is_known = box_name in scene.get_known_object_names()
			
			if (box_is_attached == is_attached) and (box_is_known == is_known):
				return True
				
			#sleep to promote other threads
			rospy.sleep(0.1)
			secondary_time = rospy.get_time()
			
		#if while loop exited without returning
		return False
	
	def add_box(self,name,pose, dims, timeout=4):
		box_name = name
		scene = self._scene

		## First, create an message object of type pose to deifne box positions
		box_pose = geometry_msgs.msg.PoseStamped()
		#box_pose.header.frame_id=self._armrobot.get_planning_frame()
		box_pose.header.frame_id = "world"
		box_pose.header.stamp = rospy.Time.now()

		#set the position massage arguments
		box_pose.pose.position.x = pose[0]
		box_pose.pose.position.y = pose[1]
		box_pose.pose.position.z = pose[2]
		
		#creating quaternion from (rpy)
		orientation_ = quaternion_from_euler(pose[3], pose[4], pose[5])
		
		box_pose.pose.orientation.x = orientation_[0]
		box_pose.pose.orientation.y = orientation_[1]
		box_pose.pose.orientation.x = orientation_[2]
		box_pose.pose.orientation.w = orientation_[3]
		#Add the box in the scene
		scene.add_box(box_name, box_pose, size=(dims[0], dims[1], dims[2]))
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

		#Set the box name for box_name of the class
		self.box_name=box_name
		#Check if the box is added sucessfully
		return self.wait_for_state_update(name, box_is_known=True, timeout=timeout)
		
		
	def attach_box(self,name, timeout=4):
		box_name = name
		robot = self._robot
		scene = self._scene
		eef_link = self._eef_link

		grasping_group = self._grasping_group
		touch_links = robot.get_link_names(group=grasping_group)
		rospy.loginfo('\033[95m' + "Touch Link: {}".format(touch_links) + '\033[0m')
		scene.attach_box(eef_link, box_name, touch_links=touch_links)
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

		# We wait for the planning scene to update.
		return self.wait_for_state_update(name,box_is_known=False, box_is_attached=True,  timeout=timeout)

	def detach_box(self,name, timeout=4):
		box_name = name
		scene = self._scene
		eef_link = self._eef_link

		scene.remove_attached_object(eef_link, name=box_name)
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html

		return self.wait_for_state_update(name, box_is_known=True, box_is_attached=False, timeout=timeout)
        
	def remove_box(self,name, timeout=4):
		box_name = name
		scene = self._scene
		scene.remove_world_object(box_name)
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
		#https://docs.ros.org/en/noetic/api/moveit_commander/html/planning__scene__interface_8py_source.html
		return self.wait_for_state_update(name, box_is_known=False, box_is_attached=False,  timeout=timeout)
            
	# Destructor
	def __del__(self):
		moveit_commander.roscpp_shutdown()
		rospy.loginfo('\033[94m' + "Object of class ArmRobot Deleted." + '\033[0m')

def main():
	#Initialize a ROS Node
	rospy.init_node('add_attach_detach_objects_in_Rviz', anonymous=True)

	robotArm = ArmRobot("arm_group", "gripper_group")
	robotArm.add_box("Table$1",pose=[0.2,0,-0.01,0.0,0.0,0.0],dims=[0.2,0.5,0.01])
	robotArm.add_box("Table$2",pose=[-0.2,0,-0.01,0.0,0.0,0.0],dims=[0.2,0.5,0.01])
	robotArm.add_box("package$1",pose=[0.2,0,0.0,0.0,0.0,0.0],dims=[0.05,0.05,0.05])
	#robotArm.add_box("package$2",0.00,0.6318,0.015,0.03,0.03,0.03)
	
	while not rospy.is_shutdown():
		task = input("Enter add to add object, attch to attach object, detach to detach object and remove to remove object: ")            
		if task == "attach":
			response = robotArm.attach_box("package$1")
			if response == True:
				rospy.loginfo('\033[32m' + "Attached Object in Rviz: {}".format(robotArm.box_name) + '\033[0m')
		if task == "detach":
			response = robotArm.detach_box("package$1")
			if response == True:
				rospy.loginfo('\033[32m' + "Detached Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
		if task == "remove":
			response = robotArm.remove_box("package$1")
			if response == True:
				rospy.loginfo('\033[32m' + "Removed Object from Rviz: {}".format(robotArm.box_name) + '\033[0m')
		if task == "x":
		    break
	#rospy.spin()
	quit()
	#delete the robotArm object at the end of code
	del robotArm
    
if __name__ == '__main__':
    main()      
