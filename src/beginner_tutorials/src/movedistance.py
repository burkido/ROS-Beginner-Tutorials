#!/usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/movedistance.py


import rospy
import sys
from geometry_msgs.msg import Twist

#v:linear, w:angular
#v=wr

node_id=str(sys.argv[1])
nodename="/turtle" + node_id

def move_task():
	rospy.init_node('movenode', anonymous=True)
	#pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)	#/turtle/cmd_vel'in sebebi subs type bu oldugu icin 
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg=Twist()
	speed=0.3
	is_forward=1
	if(is_forward == 1):
		vel_msg.linear.x=abs(speed)
	else:
		vel_msg.linear.x=-abs(speed)
	distance=3

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

if __name__=="__main__":
	try:
		move_task()
	except rospy.ROSInterruptException:
		pass






















