#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import random

rospy.init_node('velocity_publisher_node')
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)

rate = rospy.Rate(1000)

velocity_v = Twist()

while not rospy.is_shutdown():
    velocity_v.linear.x = random.random()
    velocity_v.angular.z = random.random()
    pub.publish(velocity_v)
    rate.sleep()