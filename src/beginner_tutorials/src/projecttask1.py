#!/usr/bin/env python

import rospy
import sys
#import threading
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
import tf
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

vel_msg = Twist()
vel_msg.linear.x = 0.5
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0

class Point():
	'''
	This class is used to represent a point in the map
    '''
	def __init__(self, x, y):
		self.x = x
		self.y = y

class Turtlebot:
	
	def __init__(self, nodename, nodename_to_follow):
		'''
		This class is used to control the robot
		params:
			nodename: the name of the robot
			nodename_to_follow: the name of the robot to follow
		fields:
			traffic_publisher: the publisher to publish the stop and go statuses to the robots
			traffic_subscriber: the subscriber to subscribe the stop and go statuses from the robots
			vel_publisher: the publisher to publish the velocity
			pose_subscriber: the subscriber to subscribe the pose
			location_subscriber: the subscriber to subscribe the location of the robot to follow
			scan_subscriber: the subscriber to subscribe the laser scan
   			odom: the odometry message
			pose: the pose of the robot
			rate: the rate of the robot
			roll: the roll of the robot
			pitch: the pitch of the robot
			yaw: the yaw of the robot
			point_to_go: the point to go in the map. It is initialized to (0,0) initilly. It is updated when the robot to follow reaches a new point
			should_stop: a boolean variable to indicate whether the robot should stop or not
			message_to_traffic: the message to send to the other robots
			is_first_to_start: a boolean variable to indicate whether the robot is the first to start or not
    	'''
		rospy.init_node('robotchange', anonymous=True)
		self.traffic_publisher = rospy.Publisher(nodename + '/words', String, queue_size=10)	#topic_name
		self.traffic_subscriber = rospy.Subscriber(nodename_to_follow + '/words', String, self.traffic_listener_callback)
		self.vel_publisher = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
		self.pose_subscriber = rospy.Subscriber(nodename + '/odom', Odometry, self.update_pose)
		#self.my_location_subscriber = rospy.Subscriber(nodename + '/odom', Odometry, self.my_odom_callback)
		self.goal_location_subscriber = rospy.Subscriber(nodename_to_follow + '/odom', Odometry, self.goto_odom_callback)
		self.scan_subscriber = rospy.Subscriber(nodename + '/base_scan', LaserScan, self.start_replacement)
		self.odom = Odometry()
		self.pose = self.odom.pose.pose
		self.rate = rospy.Rate(10)
		self.roll = self.pitch = self.yaw = 0.0
		self.point_to_go = Point(0,0)
		self.should_stop = False
		self.message_to_traffic = "STOP"
		self.is_first_to_start = int(sys.argv[3])
  
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

	def move2goal(self):
		'''
		This function is used to move the robot to the goal. It takes the goal from point_to_go field.
    	'''
     
		if not self.message_to_traffic == "STOP" or self.is_first_to_start == 1:
			
			newodom = Odometry()	
			goal_pose = newodom.pose.pose
			#self.point_to_go_lock.acquire()
			print("inside move2goal" + " " + str(self.point_to_go.x) + " " + str(self.point_to_go.y))
			goal_pose.position.x = self.point_to_go.x #input("x: ")
			goal_pose.position.y = self.point_to_go.y #input("y: ")

			dist_tolerance = 0.1 #input("tolerance: ")

			#print("goal x y", goal_pose.position.x, goal_pose.position.y)
			vel_msg = Twist()
	
			while self.euclidean_distance(goal_pose) >= dist_tolerance:
				print("stop status for: " + nodename + " " + str(self.should_stop))
				print("goto location for: " + nodename + " " + str(goal_pose.position.x) + " " + str(goal_pose.position.y))
				if self.should_stop:
					vel_msg.linear.x = 0
					vel_msg.angular.z = 0
					self.vel_publisher.publish(vel_msg)
				else:
					vel_msg.linear.x = self.linear_vel(goal_pose)
					vel_msg.linear.y = 0
					vel_msg.linear.z = 0
					vel_msg.angular.x = 0
					vel_msg.angular.y = 0
					vel_msg.angular.z = self.angular_vel(goal_pose)
					print(str(self.pose.position.x), str(self.pose.position.y))
					self.vel_publisher.publish(vel_msg)			
					self.rate.sleep()
			print("reached")
  
			
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			self.vel_publisher.publish(vel_msg)
			rospy.spin()

	def publish_odom(self):
		'''
		This function is used to publish the odometry message
   		'''
		#pub = rospy.Publisher(nodename + '/odom', Odometry, queue_size=10)
		rospy.init_node('odom_publisher', anonymous=True)
		rate = rospy.Rate(10) # 10hz
	
		while not rospy.is_shutdown():
			msg = Odometry()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = "odom"
			'''msg.pose.pose.position.x = 0
	        msg.pose.pose.position.y = 0
	        msg.pose.pose.orientation.x = 0
	        msg.pose.pose.orientation.y = 0
	        msg.pose.pose.orientation.z = 0
	        msg.pose.pose.orientation.w = 0'''
	
			#publish the message
			self.vel_publisher.publish(msg)
			self.rate.sleep

	def traffic_listener_callback(self, traffic_msg):
		'''
		This function is used to listen to the traffic status. Robot should move or stop according to the traffic status
		'''
		if traffic_msg == "STOP":
			self.message_to_traffic = "STOP"
		else:
			self.message_to_traffic = "GO"
		print("inside traffic_listener_callback")
		self.traffic_publisher.publish(self.should_stop)
		print(traffic_msg)

	def start_replacement(self, msg):
		'''
		This function is used to start the replacement of the robot
  		'''
		if self.message_to_traffic == "STOP" or self.is_first_to_start == 1:
			if(msg.ranges[120] > 1.0):
				self.should_stop = False
				self.message_to_traffic = "GO"
				vel_msg.linear.x = 0.5
				vel_msg.linear.y = 0.0
				self.vel_publisher.publish(vel_msg)
			else:
				#self.send_msg_to_traffic()
				message = "STOP"
				self.traffic_publisher.publish(message)
				self.should_stop = True
				self.message_to_traffic = "STOP"
				vel_msg.linear.x = 0.0
				vel_msg.linear.y = 0.0
				self.vel_publisher.publish(vel_msg)
		vel_msg.linear.x = 0.0
		vel_msg.linear.y = 0.0
		self.vel_publisher.publish(vel_msg)

	def my_odom_callback(self, msg):
		'''
		This function is used to get the odometry of the robot
  		'''
		self.my_point = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)

	def goto_odom_callback(self, msg):
		'''
		This function is used to get the odometry of the robot to replace
		'''
		self.point_to_go = Point(msg.pose.pose.position.x, msg.pose.pose.position.y)
  
	def subscribe_odom(self):
		'''
		This function is used to subscribe to the odometry of the robot. Subscribtion depends on the robot's nodename
		'''
		rospy.init_node('odom_subscriber', anonymous=True)
	
		if nodename == 'robot_0':
			#rospy.Subscriber('robot_0/odom', Odometry, own_odom_callback)
			rospy.Subscriber('robot_1/odom', Odometry, self.odom_callback)
		elif nodename == 'robot_1':
			#rospy.Subscriber('robot_1/odom', Odometry, own_odom_callback)
			rospy.Subscriber('robot_2/odom', Odometry, self.odom_callback)
		else:
			#rospy.Subscriber('robot_2/odom', Odometry, own_odom_callback)
			rospy.Subscriber('robot_0/odom', Odometry, self.odom_callback)
	
		rospy.spin()

if __name__ == "__main__":
    
    
	nodeid = str(sys.argv[1])
	nodename = 'robot_' + nodeid
	print(nodename)
 
	node_id_to_follow = str(sys.argv[2])
	nodename_to_follow = 'robot_' + node_id_to_follow

    
	try:
		robot = Turtlebot(nodename, nodename_to_follow)
		time.sleep(0.3)
		robot.move2goal()

	except rospy.ROSInterruptException:
		pass