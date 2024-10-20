#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64

def subscriberCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I received -- %f", data.data)

def subscriberMethod():
    rospy.init_node('subscriberNode', anonymous=True)
    rospy.Subscriber("talker",Float64,subscriberCallback)
    rospy.spin()

if __name__ == '__main__':
    subscriberMethod()
