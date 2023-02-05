#! /usr/bin/env python

#Make a python node executable
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/movetogoal1.py

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class Turtlebot:

	def __init__(self):

	def update_pose(self, data):

	def euclidean_distance(self, goal_pose):

	def linear_vel(self, goal_pose, constant=1.5):

	def steering_angle(self, goal_pose):

	def angular_vel(self, goal_pose, constant=6):

	def move2goal(self):

if __name__ == "__main__":
	try:
		x = Turtlebot()
		x.move2goal()

	except rospy.ROSInterruptException:
		pass








