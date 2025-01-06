#!/usr/bin/python3

import rospy
from std_srvs.srv import Empty,EmptyResponse

def empty_response_cb(req):
    rospy.loginfo("Just a Service")
    return EmptyResponse()

if __name__ == "__main__":
    rospy.logwarn("Starting an Empty Server")
    rospy.init_node('empty_ros_service')
    s = rospy.Service('print_service', Empty, empty_response_cb)
    rospy.spin()
