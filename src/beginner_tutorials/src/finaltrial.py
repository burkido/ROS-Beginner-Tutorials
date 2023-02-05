#! /usr/bin/env python

#Make a python node executable
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/scanenv1.py

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
import math
import sys

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


vel_msg = Twist()
vel_msg.linear.x = 0.5
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0

class Location:
	def __init__(self, x, y):
		self.x = x
		self.y = y

class Robot:
	
	def __init__(self, nodename):
		rospy.init_node('turtletogoal', anonymous=True)
		
		self.vel_publisher = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10) 
		self.pose_subscriber = rospy.Subscriber(nodename + '/odom', Odometry, self.update_pose)
		self.scan_subscriber = rospy.Subscriber(nodename + '/base_scan', LaserScan, self.callback)
		self.odom = Odometry()
		self.pose = self.odom.pose.pose
		self.rate = rospy.Rate(10)
		self.roll = self.pitch = self.yaw = 0.0
		self.is_rotating = False
		self.r_value = rospy.get_param('r_' + nodename, 0.0)
		self.location = Location(sys.argv[2], sys.argv[3])
		self.direction_to_turn = int(sys.argv[4])

	def update_pose(self, data):
		self.pose = data.pose.pose
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

	def euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.position.x-self.pose.position.x),2)+pow((goal_pose.position.y-self.pose.position.y),2))

	def linear_vel(self, goal_pose, constant=0.1):
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		return atan2(goal_pose.position.y-self.pose.position.y, goal_pose.position.x-self.pose.position.x)

	def angular_vel(self, goal_pose, constant=0.5):
		return constant * (self.steering_angle(goal_pose)-self.yaw)

	def move_forward(self, rospy, pub, speed, distance):
		vel_msg = Twist()

		is_forward = True
		if is_forward:
			vel_msg.linear.x = abs(speed)
		else:
			vel_msg.linear.x = -abs(speed)

		vel_msg.angular.z = 0
		current_distance = 0
		rate = rospy.Rate(10)
		t0 = rospy.Time.now().to_sec()

		while current_distance < distance:
			pub.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_distance = speed * (t1 - t0)
			rate.sleep()

		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		pub.publish(vel_msg)
  
	def calibrate_robot(self, clockwise):
		vel_msg = Twist()
		vel_msg.linear.x = 0

		angular_speed = 30 * (math.pi / 180.0)
		relative_angle = 90 * (math.pi / 180.0)
		vel_msg.angular.z = clockwise * abs(angular_speed)
		rate = rospy.Rate(10)
		t0 = rospy.Time().now().to_sec()
		current_angle = 0

		while current_angle < relative_angle:
			print("rotating", str(current_angle))
			self.is_rotating = True
			self.vel_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)
			rate.sleep()
   
		self.move_forward(rospy, self.vel_publisher, 0.5, self.r_value)
		

		self.is_rotating = True
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.vel_publisher.publish(vel_msg)

	def rotate_robot(self, clockwise):
		vel_msg = Twist()
		vel_msg.linear.x = 0

		angular_speed = 30 * (math.pi / 180.0)
		relative_angle = 88 * (math.pi / 180.0)
		vel_msg.angular.z = clockwise * abs(angular_speed)
		rate = rospy.Rate(10)
		t0 = rospy.Time().now().to_sec()
		current_angle = 0

		while current_angle < relative_angle:
			print("rotating", str(current_angle))
			self.is_rotating = True
			self.vel_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)
			rate.sleep()
   
		self.move_forward(rospy, self.vel_publisher, 0.8, self.r_value)
		self.calibrate_robot(self.direction_to_turn * -1)
		self.direction_to_turn *= -1	

		self.is_rotating = True
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.vel_publisher.publish(vel_msg)

	def callback(self, msg):

		#print(msg.ranges[180], msg.ranges[0], nodename)
		#for i in range(len(msg.ranges)):
			#print(str(i) + ": " + str(msg.ranges[i]))
		
		if(msg.ranges[120] > 1.0):
			self.is_rotating = False
			vel_msg.linear.x = 0.5
			vel_msg.linear.y = 0.0
			self.vel_publisher.publish(vel_msg)
		else:
			if not self.is_rotating:
				print("rotate_robot is called", self.is_rotating)
				self.rotate_robot(self.direction_to_turn * -1)
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.0
			self.vel_publisher.publish(vel_msg)

		
		

if __name__ == '__main__':

	node_id = sys.argv[1] 
	nodename="robot_" + node_id
	robot = Robot(nodename)

	#rospy.init_node('projecttask2', anonymous=True)
	#pub = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
	#ub = rospy.Subscriber(nodename + '/base_scan', LaserScan, callback)
	try:
		
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
