#! /usr/bin/env python3
import rospy
from std_msgs.msg import Int32

rospy.init_node('topic_publisher', anonymous=False)
pub = rospy.Publisher('counter', Int32, queue_size=10)
rate = rospy.Rate(2)
count = Int32()
count.data = 0

while not rospy.is_shutdown():
    rospy.loginfo("counter : {}".format(count.data))
    pub.publish(count)    
    count.data += 1
    rate.sleep()
