#!/usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/rotate.py


import rospy
import sys
from geometry_msgs.msg import Twist
import math

#v:linear, w:angular
#v=wr

node_id=str(sys.argv[1])	#take argument from command line
nodename="/turtle" + node_id	#rosnode list, rosnode info /turtlesm. !Related to turtle+id

rospy.init_node('movenode', anonymous=True)
pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)

def move_forward(rospy,pub, speed, distance):
	#rospy.init_node('movenode', anonymous=True)
	#pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)	#/turtle/cmd_vel'in sebebi subs type bu oldugu icin 
	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg=Twist()
	
	is_forward=1
	if(is_forward == 1):
		vel_msg.linear.x=abs(speed)
	else:
		vel_msg.linear.x=-abs(speed)

	vel_msg.angular.z = 0.0
	current_dist = 0
	rate = rospy.Rate(10)
	t0=rospy.Time().now().to_sec()

	while current_dist < distance:
		pub.publish(vel_msg)
		t1=rospy.Time.now().to_sec()				
		current_dist=speed * (t1 - t0)
		rate.sleep()
	vel_msg.linear.x=0
	vel_msg.angular.z=0
	pub.publish(vel_msg)

def rotate_task(rospy, pub, speed, angle, clockwise, lspeed=0.0):
	#rospy.init_node('movenode', anonymous=True)
	#pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)	#/turtle/cmd_vel'in sebebi subs type bu oldugu icin 
	vel_msg=Twist()
	vel_msg.linear.x=0
	#vel_msg.angular.z=0
	angular_speed = speed * (math.pi) / 180
	vel_msg.angular.z = clockwise * abs(angular_speed)
	rate = rospy.Rate(10)
	t0=rospy.Time().now().to_sec()
	current_angle = 0
	relative_angle = angle * (math.pi) / 180
	
	while current_angle < relative_angle:
		pub.publish(vel_msg)
		t1=rospy.Time.now().to_sec()				
		current_angle=angular_speed * (t1 - t0)
		rate.sleep()
	
	vel_msg.linear.x=0
	vel_msg.angular.z=0
	pub.publish(vel_msg)
	



if __name__=="__main__":
	try:
		rospy.init_node('movenode', anonymous=True)
		pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)
		rotate_task(rospy, pub, 10, 0, 1, 0.0)
		move_forward(rospy, pub, 0.5, 3)
		rotate_task(rospy, pub, 10, 90, -1, 0.0)
		move_forward(rospy, pub, 0.5, 3)
		rotate_task(rospy, pub, 10, 90, -1, 0.0)
		move_forward(rospy, pub, 0.5, 3)

	except rospy.ROSInterruptException:
		pass






















