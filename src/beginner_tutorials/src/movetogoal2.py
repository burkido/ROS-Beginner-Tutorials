#! /usr/bin/env python

#Make a python node executable
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/movetogoal2.py

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import sys

node_id = str(sys.argv[1])
nodename="/tb3_" + node_id


goalx = float(sys.argv[2])
goaly = float(sys.argv[3])


class Turtlebot:

	def __init__(self):
		rospy.init_node('turtlegoal', anonymous=True)

		#nodename + ca nbe added
		self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)	#if there are more than 1, no need to put robot_id
		self.odom = Odometry()
		self.pose = self.odom.pose.pose
		self.rate = rospy.Rate(10)
		self.roll = self.pitch = self.yaw = 0.0

	def update_pose(self, data):
		self.pose = data.pose.pose
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)
		
	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.position.x - self.pose.position.x), 2) + pow((goal_pose.position.y - self.pose.position.y), 2))

	def linear_vel(self, goal_pose, constant=3.5): #1.5
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.y)

	def angular_vel(self, goal_pose, constant=2):	#2
		return constant * (self.steering_angle(goal_pose) - self.yaw)

	def move2goal(self):
		new_odom = Odometry()
		goal_pose = new_odom.pose.pose
		goal_pose.position.x = goalx	#float(input("x: "))
		goal_pose.position.y = goaly	#input("y: ")
		dist_tolerance = 0.1 		#input("tolerance: ")	#below 5
		vel_msg = Twist()

		while self.euclidean_distance(goal_pose) >= dist_tolerance:
			vel_msg.linear.x = self.linear_vel(goal_pose)
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = self.angular_vel(goal_pose)
			print(str(self.pose.position.x), str(self.pose.position.y))
			self.vel_publisher.publish(vel_msg)
			self.rate.sleep()
		
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.vel_publisher.publish(vel_msg)

		rospy.spin()

if __name__ == "__main__":
    
	try:
		x = Turtlebot()
		x.move2goal()

	except rospy.ROSInterruptException:
		pass








