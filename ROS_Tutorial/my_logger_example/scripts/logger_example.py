#! /usr/bin/env python3

import rospy
import random
import time

# rospy.init_node('log_demo')
# rospy.init_node('log_demo', log_level=rospy.DEBUG)
# rospy.init_node('log_demo', log_level=rospy.INFO)  # default
# rospy.init_node('log_demo', log_level=rospy.WARN)
# rospy.init_node('log_demo', log_level=rospy.ERROR)
# rospy.init_node('log_demo', log_level=rospy.FATAL)

rate = rospy.Rate(0.5)

while not rospy.is_shutdown():
    rospy.logdebug("There is a missing droid")
    rospy.loginfo("The Emperors Capuchino is done")
    rospy.logwarn("The Revels are coming time "+str(time.time()))
    exhaust_number = random.randint(1,100)
    port_number = random.randint(1,100)
    rospy.logerr("The thermal exhaust port %s, right below the main port %s", exhaust_number, port_number)
    rospy.logfatal("The DeathStar Is EXPLODING")
    rate.sleep()
    rospy.logfatal("END")