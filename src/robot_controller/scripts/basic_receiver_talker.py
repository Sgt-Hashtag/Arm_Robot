#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64

def subscriberCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I received -- %f", data.data)
    test = data.data + 1.0
    pub.publish(test)

rospy.init_node('subscriberNode')
pub=rospy.Publisher('secondarytalker', Float64, queue_size=10)
sub=rospy.Subscriber("talker",Float64,subscriberCallback)
rate=rospy.Rate(10)
rospy.spin()
rate.sleep()
