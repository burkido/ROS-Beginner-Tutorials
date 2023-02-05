#!/usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/move3.py


import rospy
import sys
from geometry_msgs.msg import Twist

#v:linear, w:angular
#v=wr

node_id=str(sys.argv[1])
nodename="/turtle" + node_id
#nodename="/moveturtle" + node_id
#nodename="/robot_" + node_id

def move_task():
	print(nodename)
	rospy.init_node('movenodee', anonymous=True)
	pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)	#/turtle/cmd_vel'in sebebi subs type bu oldugu icin 
	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg=Twist()
	vel_msg.linear.x = 0.5
	vel_msg.linear.y = 0.0
	vel_msg.linear.z = 0.0

	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0.0
	if(node_id == 1):
		vel_msg.angular.z = 0.9		
	else:
		vel_msg.angular.z = 0.0
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(vel_msg)
		rate.sleep()

if __name__=="__main__":
	try:
		move_task()
	except rospy.ROSInterruptException:
		pass



