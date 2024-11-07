#!/usr/bin/env python

#General Headers
from mimetypes import init
import sys
import copy
import math
#ROS specific includes
import rospy
import moveit_commander
from  geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import *
import tf 

def create_object(obj_id, ref_frame, dimensions, pose):
    '''
    Function used to create a `object` variable that holds
    different properties of the object you are trying to create

    Args:
        obj_id:     name for the object; 
                    used to refer to in the scene (string).
        
        ref_frame:  frame in which the `pose` of the
                    object is defined (string).

        dimensions: dimension of the object. 
                    Format - [widht, breadth, thickness] (array)

        pose:       position of the object in the `ref_frame`.
                    Format - [x, y, z, roll, pitch, yaw] (array)
                    (x, y, z) in meter
                    (roll, pitch, yaw) in radians

    Return:
        object - variable containing definition of the object created. 
    '''
    object = CollisionObject() #object variable type

    object.id = obj_id
    object.header.frame_id = ref_frame

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[0]
    object_pose.position.y = pose[1]
    object_pose.position.z = pose[2]

    #creating quaternion from (rpy)
    orientation_ = tf.transformations.quaternion_from_euler(pose[3], pose[4], pose[5])

    object_pose.orientation.x = orientation_[0]
    object_pose.orientation.y = orientation_[1]
    object_pose.orientation.z = orientation_[2]
    object_pose.orientation.w = orientation_[3]

    #Adding Object 
    object.primitive_poses = [object_pose] 
    object.operation = object.ADD
    return object

def add_items(scene, frame):
    '''
    Function to iteratively add objects to the world.
    Args:
        scene: Name of the world where you want to add the object.

        frame: reference frame in which the object is added
    
    Return:
        Returns the items that can be picked as an array.
    '''
    rospy.loginfo("-- Adding Objects --")
    height_from_ground = 0.3 

    #Define the objects
    table1 = create_object(obj_id="table1", ref_frame=frame, 
                            dimensions=[0.2, 0.5, 0.01], 
                            pose=[0.5, 0, height_from_ground, 0.0, 0.0, 0.0])

    table2 = create_object(obj_id="table2", ref_frame=frame, 
                            dimensions=[0.5, 0.2, 0.01], 
                            pose=[0, 0.5, height_from_ground, 0.0, 0.0, 0.0])
    
    #box1 = create_object(obj_id="box1", ref_frame=frame, 
     #                    dimensions=[0.1, 0.025, 0.025], 
      #                   pose=[0.5, 0, height_from_ground+(0.05/2.0), 0.0, 0.0, 0.0])
    
    # MY CODE START

    box1 = create_object(obj_id="box1", ref_frame=frame, 
                         dimensions=[0.1, 0.025, 0.025], 
                         pose=[0.5, -0.1, height_from_ground+((0.01 + 0.025)/2.0), 0.0, 0.0, 0.0])
    
    box2 = create_object(obj_id="box2", ref_frame=frame, 
                         dimensions=[0.1, 0.03, 0.035], 
                         pose=[0.5, 0, height_from_ground+((0.01 + 0.035)/2.0), 0.0, 0.0, 0.0])
    
    box3 = create_object(obj_id="box3", ref_frame=frame, 
                         dimensions=[0.1, 0.04, 0.05], 
                         pose=[0.5, 0.1, height_from_ground+((0.01 + 0.05)/2.0), 0.0, 0.0, 0.0])
    
    # MY CODE END

    #Add Objects to the scene
    scene.add_object(table1)
    scene.add_object(table2)
    scene.add_object(box1)
    # MY CODE START
    scene.add_object(box2)
    scene.add_object(box3)
    # MY CODE END
    rospy.sleep(2) #Waiting to spawn the objects in the world

def gripper_action(gripper_end, action="open"):
    '''
    Function to execute the required action of the gripper.

    Args:
        gripper_end:    group_name of gripper in the manipulator

        action:         action to perform. Allowed values ["open", "close"]

    Returns:
        None.
    '''
    print("Executing gripper action = ", action)
    
    if(action == "open"):
        gripper_end.move(gripper_end.max_bound(), True)
    elif(action == "close"):
        gripper_end.move(gripper_end.max_bound()*0.5, True)
    else:
        print("Action undefined") 

def move_to_pose(arm, goal_pose):
    '''
    Function to move the robot arm to the requested pose

    Args:
        arm:        robot's arm group that should be moved.

        goal_pose:  requested pose (in the planning frame)

    Returns:
        None
    '''
    #NOTE:
    #The planner will create a plan for the panda_link8 to reach
    #the goal pose. But since we have an end-endeffector attached to it
    #it will cause collision.

    #distance between wrist(panda_link8) and gripper_end(panda_hand_tcp)
    wrist_to_tcp = 0.103

    # rospy.logwarn("planning and moving to location")
    gripper_angle = geometry_msgs.msg.Quaternion()

    #Orientation of the tcp(panda_hand_tcp) w.r.t base frame(panda_link0) 
    quaternion = tf.transformations.quaternion_from_euler(math.radians(180), 0, math.radians(-45))

    gripper_angle.x = quaternion[0]
    gripper_angle.y = quaternion[1]
    gripper_angle.z = quaternion[2]
    gripper_angle.w = quaternion[3]

    #defining pre-grasp position
    grasp_pose = Pose()
    grasp_pose.position.x = goal_pose.position.x
    grasp_pose.position.y = goal_pose.position.y
    #adding tcp distance because the planning frame is panda_link8
    grasp_pose.position.z = goal_pose.position.z + wrist_to_tcp  
    grasp_pose.orientation = gripper_angle
    
    print("Exectuting move_to_pose ({} , {},  {})"
            .format(grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z))
    
    arm.set_goal_position_tolerance(0.001) #setting tolerence for pose 
    arm.set_pose_target(grasp_pose) #setting the pose of the end-effector

    plan = arm.go(wait=True) #move the arm to the grasp_pose.

    arm.stop() #ensures that there is no residual movement.
    arm.clear_pose_targets() #to clear the existing targets.

# MY CODE START
# Pick function
def pick_action(scene, robot, arm, gripper, pick_pose, box):
    # Gripper open
    gripper_action(gripper, action="open")
    # Sleep
    rospy.sleep(1)

    # 1) Move to pre-pick location
    # Define pre-pick pose
    pre_pick_pose = Pose()
    pre_pick_pose.position.x = pick_pose.position.x
    pre_pick_pose.position.y = pick_pose.position.y
    pre_pick_pose.position.z = pick_pose.position.z + 0.1 # Placing it above the object
    # Move
    move_to_pose(arm, pre_pick_pose)
    # Sleep
    rospy.sleep(1)

    # 2) Move to pick location
    move_to_pose(arm, pick_pose)
    # Sleep
    rospy.sleep(1)

    # 3) Close the gripper and attach the object
    # Close gripper
    gripper_action(gripper, action="close")
    # Attaching the object
    eef_frame = arm.get_end_effector_link()
    grasping_group = "panda_hand"
    touch_links= robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_frame, box, touch_links=touch_links)
    # Sleep
    rospy.sleep(1)

    # 4) Move to post-pick location
    # Define post-pick pose
    post_pick_pose = Pose()
    post_pick_pose.position.x = pick_pose.position.x
    post_pick_pose.position.y = pick_pose.position.y
    post_pick_pose.position.z = pick_pose.position.z + 0.1
    # Move
    move_to_pose(arm, post_pick_pose)
    # Sleep
    rospy.sleep(1)
    return None

# Object Place function
def place_action(scene, arm, gripper, place_pose, box):
    # 1) Move to pre-drop location
    # Define pre-drop pose
    pre_drop_pose = Pose()
    pre_drop_pose.position.x = place_pose.position.x
    pre_drop_pose.position.y = place_pose.position.y
    pre_drop_pose.position.z = place_pose.position.z + 0.001
    # Move
    move_to_pose(arm, pre_drop_pose)
    # Sleep
    rospy.sleep(1)

    # 2) Move to drop location
    move_to_pose(arm, place_pose)
    # Sleep
    rospy.sleep(1)

    # 3) Open the gripper and detach the object
    # Open gripper
    gripper_action(gripper, action="open")
    # Detaching the object
    eef_frame = arm.get_end_effector_link()
    scene.remove_attached_object(eef_frame, name=box)
    # Sleep
    rospy.sleep(1)

    # 4) Move to post-drop location
    # Define post-drop pose
    post_drop_pose = Pose()
    post_drop_pose.position.x = place_pose.position.x
    post_drop_pose.position.y = place_pose.position.y
    post_drop_pose.position.z = place_pose.position.z + 0.1
    # Move
    move_to_pose(arm, post_drop_pose)
    # Sleep
    rospy.sleep(1)
    return None

# MY CODE END

def initialize_robot_params():
    '''
    Function to initialize some robot parameters.

    Args:
        None
    
    Returns:
        scene :             object variable to the world that is being simulated

        robot_commander :   object variable that command the robot

        gripper :           object variable that points to the gripper in the robot

        move_group :        object varibale that point to the arm(links) of the robot
    '''
    #Getting data about the world
    rospy.loginfo("--- MoveIt APIs initialising ---")
    scene = moveit_commander.PlanningSceneInterface() 
    robot_commander = moveit_commander.RobotCommander()
    gripper = robot_commander.get_joint('panda_finger_joint1')

    rospy.sleep(3) #essential to intialize the API's

    group_name = "panda_arm" #group name of the robot arm we are using
   
    move_group = moveit_commander.MoveGroupCommander(group_name)
    move_group.set_goal_position_tolerance(1E-2)
    move_group.set_goal_orientation_tolerance(1E-3)
    move_group.set_planning_time(10) #setting planning time in seconds
    move_group.set_planner_id("RRTConnectkConfigDefault") #setting planner to use

    return scene, robot_commander, gripper, move_group

def main():
    '''
    Function to Intialize the Node
    '''
    rospy.loginfo("--- Starting initialising a robotics node ---")
    
    #Initializing movit API
    moveit_commander.roscpp_initialize(sys.argv)

    #Initializing the ROS node
    rospy.init_node('one_object_manipulation', anonymous=True) 
    
    #Intializing robot params
    scene, robot_commander, gripper, arm = initialize_robot_params()

    # Getting some robot details
    rospy.logwarn("--- Fetching Information about the robot ---")
    planning_frame = arm.get_planning_frame()
    eef_frame = arm.get_end_effector_link()
  
    rospy.loginfo('Planning frame: '+ planning_frame)
    rospy.loginfo("End effector link: "+ eef_frame)
    
    #removing existing objects in the world
    scene.remove_world_object() #empty argument will remove all objects from the world

    # or you can use the following 3 lines to remove the objects one-by-one.
    # scene.remove_world_object("table1")
    # scene.remove_world_object("table2")
    # scene.remove_world_object("box1")
    
    #Adding objects to the scene
    add_items(scene, planning_frame)
    #Simulating Gripper actions
    rospy.loginfo("------------------")    
    
    # COMMENTED OUT BY ME - START
    #rospy.loginfo("Closing Gripper")
    #gripper_action(gripper, action="close")
    
    #rospy.loginfo("Openning Gripper")
    #gripper_action(gripper, action="open")

    #rospy.loginfo("------------------")
    
    #rospy.loginfo("Moving to pose1")
    #pose1 = Pose()
    #pose1.position.x = 0.5 #x positin in base frame
    #pose1.position.y = 0.0
    #pose1.position.z = 0.5
    #move_to_pose(arm, pose1) #sending goal pose

    #rospy.loginfo("Moving to pose2")
    #pose2 = copy.deepcopy(pose1) #making a copy of the pose
    #pose2.position.x += -1 #creating a different x value
    #move_to_pose(arm, pose2) #sending the goal pose
    # COMMENT OUT ZONE ENDS

    # MY CODE START
    # Pick box1
    rospy.loginfo("Picking object 1")
    pick_pose_1 = Pose()
    pick_pose_1.position.x = 0.5
    pick_pose_1.position.y = -0.1
    pick_pose_1.position.z = 0.3+((0.01+0.025+0.02)/2.0)
    pick_action(scene, robot_commander, arm, gripper, pick_pose_1, box="box1")
    # Place box1
    place_pose_1 = Pose()
    place_pose_1.position.x = -0.15
    place_pose_1.position.y = 0.5
    place_pose_1.position.z = 0.3+((0.01+0.025+0.02)/2)
    place_action(scene, arm, gripper, place_pose_1, box="box1")

    # Pick box2
    rospy.loginfo("Picking object 2")
    pick_pose_2 = Pose()
    pick_pose_2.position.x = 0.5 
    pick_pose_2.position.y = 0.0
    pick_pose_2.position.z = 0.3+((0.01+0.035+0.02)/2.0)
    pick_action(scene, robot_commander, arm, gripper, pick_pose_2, box="box2")
    # Place box2
    place_pose_2 = Pose()
    place_pose_2.position.x = 0.0
    place_pose_2.position.y = 0.5
    place_pose_2.position.z = 0.3+((0.01+0.035+0.02)/2.0)
    place_action(scene, arm, gripper, place_pose_2, box="box2")

    # Pick box3
    rospy.loginfo("Picking object 3")
    pick_pose_3 = Pose()
    pick_pose_3.position.x = 0.5 
    pick_pose_3.position.y = 0.1
    pick_pose_3.position.z = 0.3+((0.01+0.05+0.02)/2.0)
    pick_action(scene, robot_commander, arm, gripper, pick_pose_3, box="box3")
    # Place box3
    place_pose_3 = Pose()
    place_pose_3.position.x = 0.15
    place_pose_3.position.y = 0.5
    place_pose_3.position.z = 0.3+((0.01+0.05+0.02)/2.0)
    place_action(scene, arm, gripper, place_pose_3, box="box3")

    # MY CODE ENDS
    
if __name__ == '__main__':
    #starting point of the code
    main()