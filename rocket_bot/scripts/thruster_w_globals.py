#!/usr/bin/env python

"""Example of a ROS node that uses globals to store the
values received from sensore callbacks.  Not very good style.

Author: Nathan Sprague
Version: 9/8/2015

"""
import rospy

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

# globals
LOCATION = None

# This function will be called every time a new location message.
def location_callback(loc_msg):
    """ loc_msg will be of type Point """

    global LOCATION # prevents creation of a local LOCATION variable.
    LOCATION = loc_msg


def start():
    """ Initialize the node, and execute the main loop """

    # Turn this into an official ROS node named approach
    rospy.init_node('thruster')

    # Subscribe to the location topic.  From now on location_callback
    # will be called every time a new location message is published.
    rospy.Subscriber('location', Point, location_callback)

    # Create a publisher object for sending vector messages to the
    # RocketBot's thrust topic.
    thrust_pub = rospy.Publisher('thrust', Vector3, queue_size=1)

    # Create a Vector message object.
    thrust = Vector3()

    # Spin until the first location message is available.
    while LOCATION is None and not rospy.is_shutdown():
        rospy.sleep(.1)

    # Try to stay above this altitude.
    target_altitude = 100.0

    # Rate object used to make the main loop execute at 10hz.
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        if LOCATION.y < target_altitude:
            thrust.y = 100.0
            rospy.loginfo("THRUSTERS ENGAGED")
        else:
            thrust.y = 0.0

        thrust_pub.publish(thrust)
        rate.sleep()           # Pause long enough to maintain correct rate.


# This is how we usually call the main method in Python. 
if __name__ == "__main__":
    start()
