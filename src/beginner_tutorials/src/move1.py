#!/usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/move1.py


import rospy
from geometry_msgs.msg import Twist

#v:linear, w:angular
#v=wr
def move_task():
	print("broski")
	rospy.init_node('movenode', anonymous=True)
	pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)	#/turtle/cmd_vel'in sebebi subs type bu oldugu icin 
	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)	#/turtle/cmd_vel'in sebebi subs type bu oldugu icin 
	#pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	vel_msg=Twist()
	vel_msg.linear.x = 0.5
	vel_msg.linear.y = 0.0
	vel_msg.linear.z = 0.0

	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0.0
	vel_msg.angular.z = 0.9
	rate = rospy.Rate(10)
	while not rospy.is_shutdown():
		pub.publish(vel_msg)
		rate.sleep()

if __name__=="__main__":
	try:
		move_task()
	except rospy.ROSInterruptException:
		pass



