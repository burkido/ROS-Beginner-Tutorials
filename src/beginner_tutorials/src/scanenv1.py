#! /usr/bin/env python

#Make a python node executable
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/scanenv1.py

import rospy
import sys
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


node_id = str(sys.argv[1])
nodename="robot_" + node_id
#nodename="tb3_" + node_id	#for gazebo
print('nodename:' + nodename)

vel_msg = Twist()
vel_msg.linear.x = 0.5
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0


def callback(msg):	#default value without obstacle is 5
	obstacle = False
	
	for i in range(len(msg.ranges)):
		print(str(i) + ": " + str(msg.ranges[i]))

	if(msg.ranges[120] > 0.5):
		vel_msg.linear.x = 0.5
		vel_msg.linear.y = 0.0

	else:
		vel_msg.linear.x = 0.0
		vel_msg.linear.y = 0.0
	
	pub.publish(vel_msg)

rospy.init_node(nodename, anonymous=True)
pub = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
#sub = rospy.Subscriber(nodename + '/base_scan', LaserScan, callback)
sub = rospy.Subscriber(nodename + '/base_scan', LaserScan, callback)

rospy.spin()
