#!/usr/bin/env python3

import rospy

rospy.init_node('my_first_python_node')
rospy.loginfo("This node has been started")

rate = rospy.Rate(5)

while not rospy.is_shutdown():
    rospy.loginfo("Hello")
    rate.sleep()
