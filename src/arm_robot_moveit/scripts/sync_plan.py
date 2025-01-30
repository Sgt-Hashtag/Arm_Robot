#!/usr/bin/env python3
# encoding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit


kit = None


def callback(data):
    global kit
    # rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        if radians_to_angles>180:
            radians_to_angles = 180
        if index==1:
            radians_to_angles = 90 - radians_to_angles     
        if index>1 and index<4 :
            radians_to_angles = 90 + radians_to_angles
        if index==5:
            radians_to_angles = 180 + radians_to_angles  
        
        data_list.append(round(radians_to_angles,1))
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    # mc.send_angles(data_list, 25)
    kit.servo[0].angle = data_list[0]
    kit.servo[1].angle = data_list[1]
    kit.servo[2].angle = data_list[2]
    kit.servo[3].angle = data_list[3]
    kit.servo[4].angle = data_list[4]
    kit.servo[5].angle = data_list[5]


def listener():
    rospy.init_node("motorcontrolNode", anonymous=True)
    time.sleep(0.05)
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    kit = ServoKit(channels=16)
    m = 0
    kit.servo[0].actuation_range = 180
    kit.servo[0].set_pulse_width_range(600, 2600)
    kit.servo[1].actuation_range = 180
    kit.servo[1].set_pulse_width_range(600, 2600)
    kit.servo[2].actuation_range = 180
    kit.servo[2].set_pulse_width_range(600, 2600)
    kit.servo[3].actuation_range = 180
    kit.servo[3].set_pulse_width_range(600, 2600)
    kit.servo[4].actuation_range = 180
    kit.servo[4].set_pulse_width_range(600, 2600)
    kit.servo[5].actuation_range = 180
    kit.servo[5].set_pulse_width_range(600, 2600)
    kit.servo[6].actuation_range = 180
    kit.servo[6].set_pulse_width_range(600, 2600)
   
    kit.servo[0].angle = 90.0
    kit.servo[1].angle = 135.0
    kit.servo[2].angle = 180.0
    kit.servo[3].angle = 135.0
    kit.servo[4].angle = 15.0 
    kit.servo[5].angle = 165.0
    listener()
