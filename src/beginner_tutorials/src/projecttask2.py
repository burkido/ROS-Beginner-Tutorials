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

vel_msg = Twist()
vel_msg.linear.x = 0.5
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0

class Robot:
	'''
	This class is used to control the robot
	'''
	def __init__(self, nodename):
		'''
		This class is used to control the robot
		params:
			nodename: the name of the robot
		fields:
			vel_publisher: the publisher to publish the velocity
			pose_subscriber: the subscriber to subscribe the pose
			scan_subscriber: the subscriber to subscribe the laser scan
			odom: the odometry message
			pose: the pose of the robot
			rate: the rate of the node
			roll: the roll of the robot
			pitch: the pitch of the robot
			yaw: the yaw of the robot
			is_rotating: the flag to indicate if the robot is rotating
			current_rotate_count: the current rotate count
			max_rotate_count: the max rotate count
			is_map_done: the flag to indicate if the map is done
			r_value: the r value of the robot
			direction_to_turn: the direction to turn
  		'''
		
		rospy.init_node('turtletogoal', anonymous=True)
		self.vel_publisher = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10) 
		self.pose_subscriber = rospy.Subscriber(nodename + '/odom', Odometry, self.update_pose)
		self.scan_subscriber = rospy.Subscriber(nodename + '/base_scan', LaserScan, self.callback)
		self.odom = Odometry()
		self.pose = self.odom.pose.pose
		self.rate = rospy.Rate(10)
		self.roll = self.pitch = self.yaw = 0.0
		self.is_rotating = False
		self.current_rotate_count = 1
		self.max_rotate_count = int(sys.argv[5])
		self.is_map_done = False
		self.r_value = rospy.get_param('r_' + nodename, 0.0)
		#self.location = Location(sys.argv[2], sys.argv[3])
		self.direction_to_turn = int(sys.argv[4])

	def update_pose(self, data):
		'''
		This function is used to update the pose of the robot while it is moving
		params:
			data: the odometry message
  		'''
		self.pose = data.pose.pose
		orientation_q = self.pose.orientation
		orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
		(self.roll, self.pitch, self.yaw) = euler_from_quaternion (orientation_list)

	def euclidean_distance(self, goal_pose):
		'''
		This function is used to calculate the euclidean distance between the current pose and the goal pose
    	'''
		return sqrt(pow((goal_pose.position.x-self.pose.position.x),2)+pow((goal_pose.position.y-self.pose.position.y),2))

	def linear_vel(self, goal_pose, constant=0.1):
		'''
		This function is used to calculate the linear velocity
  		'''
		return constant * self.euclidean_distance(goal_pose)

	def steering_angle(self, goal_pose):
		'''
		This function is used to calculate the steering angle
		'''
		return atan2(goal_pose.position.y-self.pose.position.y, goal_pose.position.x-self.pose.position.x)

	def angular_vel(self, goal_pose, constant=0.5):
		'''
		This function is used to calculate the angular velocity
		'''
		return constant * (self.steering_angle(goal_pose)-self.yaw)

	def move_forward(self, pub, speed, distance):
		'''
		This function is used to move the robot forward with given speed and distance. It creates own Twist at this stage.
		'''
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
		'''
		This function is used to calibrate the robot after rotation. It rotates the robot with spesific speed and angle in gicen clockwise.
		This function will run after first rotation of the robot when it encounters with an obstacle.
  		'''
     
		vel_msg = Twist()
		vel_msg.linear.x = 0

		angular_speed = 30 * (math.pi / 180.0)
		relative_angle = 86 * (math.pi / 180.0)
		vel_msg.angular.z = clockwise * abs(angular_speed)
		rate = rospy.Rate(10)
		t0 = rospy.Time().now().to_sec()
		current_angle = 0
  
		while current_angle < relative_angle:
			
			self.is_rotating = True
			self.vel_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)
			rate.sleep()
   
		self.current_rotate_count += 1
		print("current_rotate_count of: " + str(nodename) + " " + str(self.current_rotate_count))

		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.vel_publisher.publish(vel_msg)

	def rotate_robot(self, clockwise):
		'''
		This function is used to rotate the robot with spesific speed and angle in gicen clockwise. This function will run after
		it encounters with an obstacle
  		'''
     
		vel_msg = Twist()
		vel_msg.linear.x = 0
	
		self.is_rotating = True

		angular_speed = 30 * (math.pi / 180.0)
		relative_angle = 88 * (math.pi / 180.0)
		vel_msg.angular.z = clockwise * abs(angular_speed)
		rate = rospy.Rate(10)
		t0 = rospy.Time().now().to_sec()
		current_angle = 0

		while current_angle < relative_angle:
			self.vel_publisher.publish(vel_msg)
			t1 = rospy.Time.now().to_sec()
			current_angle = angular_speed * (t1 - t0)
			rate.sleep()

		self.current_rotate_count += 1
  
		print("current_rotate_count of: " + str(nodename) + " " + str(self.current_rotate_count))
   
		self.move_forward(self.vel_publisher, 0.8, self.r_value)

		if(self.current_rotate_count == self.max_rotate_count):
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.0
			self.vel_publisher.publish(vel_msg)
			self.is_map_done = True
			self.is_rotating = False
			self.current_rotate_count = 0
			return
  
		self.calibrate_robot(self.direction_to_turn * -1)
		self.direction_to_turn *= -1	

		self.is_rotating = True
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self.vel_publisher.publish(vel_msg)
		
	def callback(self, msg):
		'''
		This function is used to get the laser scan data and check if there is an obstacle in front of the robot.
  		If there is an obstacle it will stop and rotate the robot.
  		'''

		if(msg.ranges[120] > 1.0):
			self.is_rotating = False
			vel_msg.linear.x = 0.5
			vel_msg.linear.y = 0.0
			self.vel_publisher.publish(vel_msg)
		else:
			if not self.is_rotating:
				if(self.current_rotate_count == self.max_rotate_count):
					print("current_rotate_count of: " + str(nodename) + " " + str(self.current_rotate_count))
					vel_msg.linear.x = 0.0
					vel_msg.linear.y = 0.0
					self.vel_publisher.publish(vel_msg)
					return
				self.rotate_robot(self.direction_to_turn * -1)
			vel_msg.linear.x = 0.0
			vel_msg.linear.y = 0.0
			self.vel_publisher.publish(vel_msg)
		
if __name__ == '__main__':
 
	try:
		node_id = sys.argv[1] 
		nodename="robot_" + node_id
		print("nodename: " + nodename)
		robot = Robot(nodename)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass
