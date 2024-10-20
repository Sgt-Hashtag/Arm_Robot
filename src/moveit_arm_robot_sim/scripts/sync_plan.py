#!/usr/bin/env python3
# encoding=utf-8
import time
import math
import rospy
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit


kit = None


def callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "%s", data)
    data_list = []
    for index, value in enumerate(data.position):
        radians_to_angles = round(math.degrees(value), 2)
        if radians_to_angles<0:
            radians_to_angles = 90 - radians_to_angles
        if radians_to_angles>180 and radians_to_angles<181:
            radians_to_angles = 180
        data_list.append(radians_to_angles)
        
    rospy.loginfo(rospy.get_caller_id() + "%s", data_list)
    # mc.send_angles(data_list, 25)
    kit.servo[0].angle = data_list[0]
    kit.servo[1].angle = data_list[1]
    kit.servo[2].angle = data_list[2]
    kit.servo[3].angle = data_list[3]


def listener():
    global kit
    rospy.init_node("mycobot_reciver", anonymous=True)
    kit = ServoKit(channels=16)
    time.sleep(0.05)
    
    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == "__main__":
    listener()