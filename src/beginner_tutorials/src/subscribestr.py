#!/usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/subscribestr.py

import rospy
from std_msgs.msg import String


rospy.init_node('stringsub')

def callback(msg):
	print(msg)

#1st param: topic name
#2nd param: message type
#3rd param: callback function
sub = rospy.Subscriber('/words', String, callback)	#they communicate over topic. Note: /words is topic in this case

rospy.spin()



