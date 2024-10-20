#!/usr/bin/env python3

#rostopic pub -1 /angle_val std_msgs/Int64 '45'

import rospy
from std_msgs.msg import String, Int64
from time import *
from adafruit_servokit import ServoKit

def subscriberCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I received -- %f", data.data)
    global m
    rospy.loginfo(m) 
    for i in range(m):
    	kit.servo[i].angle = data.data

def subscriberMethod():
    rospy.init_node('motorcontrolNode')
    rospy.Subscriber("angle_val",Int64,subscriberCallback)
    rospy.spin()

if __name__ == '__main__':
    kit = ServoKit(channels=16)
    m = 0
    for i in range(4):
    	kit.servo[i].angle = 0
    	m=m+1
    subscriberMethod()                        
