#!/usr/bin/python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/projecttask2.py

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random
import sys
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import LaserScan


node_id = str(sys.argv[1])
nodename = "robot_" + node_id
print('nodename:' + nodename)

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

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        
    def update_point(self, x, y):
        self.x = x
        self.y = y

class Robot:
    
    def __init__(self):
        rospy.init_node('finalnodes', anonymous=True)
        self.vel_publisher = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber(nodename + '/odom', Odometry, self.update_pose)
        self.odom = Odometry()
        self.pose = self.odom.pose.pose
        self.rate = rospy.Rate(10)
        self.roll = self.pitch = self.yaw = 0.0
        self.point= Point(sys.argv[2], sys.argv[3])
        self.current_direction = 'Up'
        self.map_finished = False
        
    def update_pose(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.position.x-self.pose.position.x),2)+pow((goal_pose.position.y-self.pose.position.y),2))
    
    def linear_vel(self, goal_pose, constant=0.1):
        return constant * self.euclidean_distance(goal_pose)
    
    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y-self.pose.position.y, goal_pose.position.x-self.pose.position.x)
    
    def angular_vel(self, goal_pose, constant=0.5):
        return constant * (self.steering_angle(goal_pose)-self.yaw)
    
    def move2goal(self):
        newodom = Odometry()
		#print("new x y", newodom.pose.pose.position.x, newodom.pose.pose.position.y)		
        goal_pose = newodom.pose.pose
        goal_pose.position.x = 2 #input("x: ")
        goal_pose.position.y = 2 #input("y: ")
        dist_tolerance = 0.1 #input("tolerance: ")
		
        print("goal x y", goal_pose.position.x, goal_pose.position.y)
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

def move_forward(
        self,
        rospy,
        pub,
        speed,
        distance,
        ):
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
   
vel_msg = Twist()
vel_msg.linear.x = 0.5
vel_msg.linear.y = 0.0
vel_msg.linear.z = 0.0
vel_msg.angular.x = 0.0
vel_msg.angular.y = 0.0
vel_msg.angular.z = 0.0
   
def callback(msg):	#default value without obstacle is 5
	obstacle = False
	
	for i in range(len(msg.ranges)):
		print(str(i) + ": " + str(msg.ranges[i]))

	if(msg.ranges[120] > 0.5):
		vel_msg.linear.x = 0.5
		vel_msg.linear.y = 0.0

	else:
		vel_msg.linear.x = 0.0
		vel_msg.linear.y = 0.0
	
	pub.publish(vel_msg)
        
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
 
if __name__ == '__main__':
    
	rospy.init_node("nodename", anonymous=True)
	pub = rospy.Publisher(nodename + '/cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)

    #sub = rospy.Subscriber(nodename + '/scan', LaserScan, callback)
	print("subscribing")
	sub = rospy.Subscriber(nodename + '/base_scan', LaserScan, callback)
	print("subscribed")
	controlrobot()
	print("done")
	rospy.spin()
	print("after spin")
    
	'''try:
        
        x = Robot()
        print('x:' + str(x.point.x))
        print('y:' + str(x.point.y))
        move_forward(x, rospy, x.vel_publisher, 0.8, 4)
        #x.move2goal()
 
    except rospy.ROSInterruptException: pass'''