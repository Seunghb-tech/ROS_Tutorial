#!/usr/bin/env python

"""Python class illustrating a style of ROS node with no explicit main
 loop.  The logic is completely event driven.

This node doesn't store incoming sensor messages.  Instead it responds
to them directly from the callback.

Author: Nathan Sprague
Version: 9/8/2015

"""
import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

class ThrusterNode(object):
    def __init__(self):
        rospy.init_node('thruster')

        rospy.Subscriber('location', Point, self.location_callback)

        self.thrust_pub = rospy.Publisher('thrust', Vector3, queue_size=1)

        self.target_altitude = 100.0
        rospy.spin()

    def location_callback(self, loc_msg):
        """ loc_msg will be of type Point """
        thrust = Vector3()

        if loc_msg.y < self.target_altitude:
            thrust.y = 100.0
            rospy.loginfo("THRUSTERS ENGAGED")
        else:
            thrust.y = 0.0

        self.thrust_pub.publish(thrust)

if __name__ == "__main__":
    ThrusterNode()

