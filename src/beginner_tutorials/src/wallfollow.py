#! /usr/bin/env python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/wallfollow.py

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

move = Twist()
move.linear.x = 0.5
move.angular.z = 0.0

minfrontdistance = 0.3
stopdist = 0.3
stop = 0
obstacle = False
speed = 0.5
AVOID_SPEED = 0.05

minleft = 10000000.0
minright = 10000000.0


#nodeid = str(sys.argv[1])
#nodename = 'turtle'+nodeid #'tb3_' + nodeid   #for Gazebo

node_id = str(sys.argv[1])
nodename = "robot_" + node_id
print('nodename:' + nodename)

rospy.init_node("nodename", anonymous=True)
#pub = rospy.Publisher(nodename+'/cmd_vel', Twist, queue_size=10)
pub = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10)

def rotateRobot(direction):
	#print("Let's turn your robot")
	move.linear.x = 0
	move.angular.z = 0.1 * direction
	pub.publish(move)

def obstacleroutine(minleft, minright):
	if stop == 1:
		speed = AVOID_SPEED
	if minleft < minright:
		rotateRobot(+1)
	elif minleft >= minright:
		rotateRobot(-1)

def callback(msg):
	#print("msg.ranges[0]")
	global obstacle 
	global minleft
	global minright
	obstacle = False
	size = len(msg.ranges)

	for i in range(80, 130):
		print(i, msg.ranges[0], msg.ranges[180])
		if 0 <= i <= 29 or 330 <= i < 360:
			if float(msg.ranges[i]) < minfrontdistance and float(msg.ranges[i]) != 0.0:		
				#print(i, msg.ranges[i])			
				obstacle = True

		#print(i, msg.ranges[i], stopdist)
		if float(msg.ranges[i]) <= stopdist:
			stop = 1	
			move.linear.x = 0.0
			move.angular.z = 0.0
			#print("moving")	
			pub.publish(move)
			#rate.sleep()		

		if i < size/2:
			minleft = min( minleft, float(msg.ranges[i]))
		else:      
			minright = min( minright, float(msg.ranges[i]))

def rotate():
	global obstacle

	vel_msg = Twist()

	# Receiving the user's input
	#print("Let's rotate your robot")
	clockwise = True #input("Clockwise?: ") #True or false

	#Converting from angles to radians
	angular_speed = 20.0*2*3.14/360
	relative_angle = 90.0*2*3.14/360

	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0

	if clockwise:
		vel_msg.angular.z = -abs(angular_speed)
	else:
		vel_msg.angular.z = abs(angular_speed)

	t0 = rospy.Time.now().to_sec()
	current_angle = 0
	r = rospy.Rate(10)
	while(obstacle != False):
		pub.publish(vel_msg)
		t1 = rospy.Time.now().to_sec()
		current_angle = angular_speed*(t1-t0)
		r.sleep()

	vel_msg.linear.x=0
	vel_msg.angular.z = 0
	pub.publish(vel_msg)

def controlrobot():
	global obstacle
	while obstacle != True:
		#print("moving")
		move.linear.x = 0.1
		move.angular.z = 0.0			
		pub.publish(move)
		rate.sleep()

	rotate()
	controlrobot()

#sub = rospy.Subscriber('/base_scan', LaserScan, callback)
#sub = rospy.Subscriber(nodename+'/scan', LaserScan, callback)
sub = rospy.Subscriber(nodename + '/base_scan', LaserScan, callback)
controlrobot()
rospy.spin()

	



