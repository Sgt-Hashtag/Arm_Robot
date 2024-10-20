#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int64
from adafruit_servokit import ServoKit


def publisherMethod():
    pub= rospy.Publisher('talker', String, queue_size=10)
    rospy.init_node('publisherNode', anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
    	publishString = f"{kit.servo[0].angle},{kit.servo[1].angle},{kit.servo[2].angle},{kit.servo[3].angle}" 	   
    	rospy.loginfo("Data is being sent")
    	rospy.loginfo(publishString)
    	pub.publish(publishString)
    	rate.sleep()

if __name__ == '__main__':
    try:
    	kit = ServoKit(channels=16)
    	publisherMethod()
    except rospy.ROSInterruptException:
    	pass
