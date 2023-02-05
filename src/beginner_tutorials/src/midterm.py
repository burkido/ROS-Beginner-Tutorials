#!/usr/bin/python
#chmod u+x ~/catkin_ws/src/beginner_tutorials/src/midterm.py

import rospy
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import random
import sys
from math import pow, atan2, sqrt
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Neighbor:
    
    ''' This class is used to store the neighbors of a cell. The neighbors are stored in clockwise order.
    
    Attributes
    ----------
    - up : value of the neighbor above the cell
    - right : value of the neighbor to the right of the cell
    - down : value of the neighbor below the cell
    - left : value of the neighbor to the left of the cell
    '''

    def __init__(  # directions in clockwise order up -> right -> down -> left. '-1' means no neighbor
        self,
        up,
        right,
        down,
        left,
        ):
        self.up = up
        self.right = right
        self.down = down
        self.left = left

class Cell:
    
    ''' This class represents each cell in the stage_map. Each cell has an id, a boolean value for whether
        it is an obstacle or not, a reward value, and a direction value. Also there is 4x4 square in each cell.
        
        Attributes
        ----------
        - id: id of the cell
        - is_obstacle: boolean value for whether it is an obstacle or not
        - reward: int value if the cell has a reward
        - direction: neighbor class for up, right, down, left direction.
    '''

    def __init__(
        self,
        id,
        is_obstacle,
        reward,
        direction
        ):
        self.id = id
        self.is_obstacle = is_obstacle
        self.reward = reward
        self.direction = direction

    def __str__(self):
        return self.id

class Robot:
    
    ''' This function is used to communicate with ROS and rotate the robot to a specific angle, calculations of formulas and holding
    robot spesific values. The angle is given in degrees.
    
    Attributes
    ----------
    - velocity_publisher: publisher to publish the velocity of the robot
    - pose_subscriber: subscriber to subscribe to the pose of the robot for Odometry data
    - odom: instance of Odometry class
    - pose: pose data from Odometry class
    - rate: hz rate to determine sleep time
    - roll: 
    - current_direction: direction of robot's faced direction
    - collected_rewards_count: number of rewards collected by robot
    - map_finished: boolean value to determine if the map is finished
    '''

    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/odom', Odometry, self.update_pose)
        self.odom = Odometry()
        self.pose = self.odom.pose.pose
        self.rate = rospy.Rate(10)
        self.roll = self.pitch = self.yaw = 0.0
        self.current_direction = 'Right'
        self.collected_rewards_count = 0
        self.map_finished = False

    def update_pose(self, data):
        self.pose = data.pose.pose
        orientation_q = self.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow(goal_pose.position.x - self.pose.position.x, 2) + pow(goal_pose.position.y - self.pose.position.y, 2))

    def linear_vel(self, goal_pose, constant=3.5):
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.position.y - self.pose.position.y, goal_pose.position.x - self.pose.position.y)

    def angular_vel(self, goal_pose, constant=2):
        return constant * (self.steering_angle(goal_pose) - self.yaw)

    def collect_reward(self):
        self.collected_rewards_count +=1

    def update_last_visited(self, cell):
        self.last_visited = cell

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

def rotate_robot(
    rospy,
    pub,
    speed,
    angle,
    clockwise,
    lspeed=0.0,
    ):
    
    ''' This function is used to communicate with ROS and rotate the robot to a specific angle, calculations of formulas and holding
    
    Parameters
    ----------
    - rospy: instance of rospy class
    - pub: publisher to publish the velocity of the robot
    - speed: speed of the robot
    - angle: angle to rotate the robot
    - clockwise: boolean value to determine if the robot should rotate clockwise or not
    - lspeed: linear speed of the robot
    '''

    vel_msg = Twist()
    vel_msg.linear.x = 0

    angular_speed = speed * (math.pi / 180.0)
    relative_angle = angle * (math.pi / 180.0)
    vel_msg.angular.z = clockwise * abs(angular_speed)
    rate = rospy.Rate(10)
    t0 = rospy.Time().now().to_sec()
    current_angle = 0

    while current_angle < relative_angle:
        pub.publish(vel_msg)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        rate.sleep()

    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    pub.publish(vel_msg)

def neighbours(direction):
    ''' This function is used to determine the neighbors of the current cell.
    
    Parameters
    ----------
    - direction: instance of direction class to represent robot's neighbours
    '''
    directions = []
    if direction.up != None:
        directions.append(direction.up)
    if direction.right != None:
        directions.append(direction.right)
    if direction.down != None:
        directions.append(direction.down)
    if direction.left != None:
        directions.append(direction.left)

    # print(directions)

    return directions

def rotate_left_and_go(
    rospy,
    robot,
    speed=10,
    angle=90,
    ):
    
    ''' This function is used to rotate the robot to the left and move forward.
    
    Parameters
    ----------
    - rospy: instance of rospy class
    - robot: instance of robot class
    - speed: speed of the robot
    - angle: angle to rotate the robot. Default value is 90 degrees
    '''
    
    rotate_robot(
        rospy,
        robot.vel_publisher,
        speed,
        88,
        1,
        0.0,
        )
    robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)

def rotate_right_and_go(
    rospy,
    robot,
    speed=10,
    angle=90,
    ):
    
    ''' This function is used to rotate the robot to the right and move forward.
    
    Parameters
    ----------
    - rospy: instance of rospy class
    - robot: instance of robot class
    - speed: speed of the robot
    - angle: angle to rotate the robot. Default value is 90 degrees
    '''
    
    rotate_robot(
        rospy,
        robot.vel_publisher,
        speed,
        89,
        -1,
        0.0,
        )
    robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)

def rotate_full_and_go(
    rospy,
    robot,
    speed=10,
    angle=180,
    ):
    
    ''' This function is used to rotate the robot 180 degress and move forward.
    
    Parameters
    ----------
    - rospy: instance of rospy class
    - robot: instance of robot class
    - speed: speed of the robot
    - angle: angle to rotate the robot. Default value is 180 degrees
    '''
    rotate_robot(
        rospy,
        robot.vel_publisher,
        speed,
        angle,
        1,
        0.0,
        )
    robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)

def find_direction(current_node, parent_node, robot):
    
    ''' This function is used to find the direction of the robot based on the current node and the parent node and current direction.
    
    Parameters
    ----------
    - current_node: robot's next node to go
    - parent_node: where the robot is come from
    - robot: instance of robot class
    '''
    
    if current_node == '00':
        return

    if robot.current_direction == 'Up':
        if current_node[0] > parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is already faced up.')
            robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)
        elif current_node[0] < parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced up. Rotating 180 degree!')
            rotate_full_and_go(rospy, robot, speed=10, angle=180)
            robot.current_direction = 'Down'
        elif current_node[1] > parent_node[1]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced up. Rotating 90 degree right!')
            rotate_right_and_go(rospy, robot, speed=10, angle=90)
            robot.current_direction = 'Right'
        elif current_node[1] < parent_node[1]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced up. Rotating 90 degree left!')
            rotate_left_and_go(rospy, robot)
            robot.current_direction = 'Left'
    elif robot.current_direction == 'Right':

        if current_node[0] > parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced right. Rotating 90 degree left!')
            rotate_left_and_go(rospy, robot, speed=10, angle=90)  # Rotate left when robot is facing right
            robot.current_direction = 'Up'
        elif current_node[0] < parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced right. Rotating 90 degree left!')
            rotate_right_and_go(rospy, robot)
            robot.current_direction = 'Down'
        elif current_node[1] > parent_node[1]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is already faced right.')
            robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)
        elif current_node[1] < parent_node[1]:
            # robot.current_direction = 'Right'
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced right. Rotating 180 degree!')
            # otate_robot(rospy, robot.vel_publisher, 10, 150, 1, 0.0)

            rotate_full_and_go(rospy, robot)
            robot.current_direction = 'Left'
    elif robot.current_direction == 'Down':

        if current_node[0] > parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced down. Rotating 180 degree!')
            rotate_robot(
                rospy,
                robot.vel_publisher,
                10,
                180,
                1,
                0.0,
                )
            robot.current_direction = 'Up'
        elif current_node[0] < parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is already faced down.')
            robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)
        elif current_node[1] > parent_node[1]:

            # robot.current_direction = 'Right'

            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced down. Rotating 90 degree left!')
            rotate_left_and_go(rospy, robot)
            robot.current_direction = 'Right'
        elif current_node[1] < parent_node[1]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node +' Robot is faced right. Rotating 90 degree right!')
            rotate_right_and_go(rospy, robot)
            robot.current_direction = 'Left'
    elif robot.current_direction == 'Left':

        if current_node[0] > parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced left. Rotating 90 degree right!')
            rotate_right_and_go(rospy, robot)
            robot.current_direction = 'Up'
        elif current_node[0] < parent_node[0]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced left. Rotating 90 degree left!')
            rotate_left_and_go(rospy, robot)
            robot.current_direction = 'Down'
        elif current_node[1] > parent_node[1]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is faced right. Rotating 180 degree.')
            rotate_robot(
                rospy,
                robot.vel_publisher,
                10,
                180,
                1,
                0.0,
                )
            robot.current_direction = 'Right'
        elif current_node[1] < parent_node[1]:
            print ('Current node: ' + current_node + ' Parent node: ' + parent_node + ' Robot is already faced left.')
            robot.move_forward(rospy, robot.vel_publisher, 0.8, 4)
            # robot.current_direction = 'Right' 
def check_cell_has_reward(graph, current_node):
    
    ''' This function checks if the current node has reward or not.
    
    Parameters
    ----------
    - graph: graph of the map
    - current_node: current node of the robot
    
    Returns
    -------
    - True if the current node has reward or false if not.
    '''
    
    if graph[current_node].reward == 1:
        return True
    else:
        return False

def collect_rewards_with_dfs(
    graph,
    current_node,
    visited,
    visited_for_backtrack,
    robot,
    total_reward_count,
    parent_node='',
    ):
    
    ''' This function collects rewards and explore the map with recursive DFS algorithm.
    
    Parameters
    ----------
    - graph: graph of the map
    - current_node: current node of the robot. This node is the node that robot will go. But in simulation it is the node that robot is in.
    - visited: list of visited nodes
    - visited_for_backtrack: list of visited nodes for backtracking
    - robot: robot object
    - total_reward_count: total reward count in the map. It takes it as a argument
    - parent_node: parent node of the current node.
    '''
    
    if parent_node == '02':     # end of the map
            find_direction('01', parent_node, robot)
            exit(1)
            
    if check_cell_has_reward(graph, current_node):
        robot.collect_reward()
        print ('Found reward at node: ' + current_node + ' Reward count: ' + str(robot.collected_rewards_count))
        if robot.collected_rewards_count == total_reward_count:
            print ('Robot has collected all rewards. Exiting!')
            exit(1)
        
    if current_node not in visited and not graph[current_node].is_obstacle and robot.map_finished == False:
        #print('Current node: ' + current_node + ' Parent node: ' + parent_node)
        find_direction(current_node, parent_node, robot)
        visited.append(current_node)
        for neighbour in neighbours(graph[current_node].direction):
            parent_node = current_node
            collect_rewards_with_dfs(
                graph,
                neighbour,
                visited,
                visited_for_backtrack,
                robot,
                total_reward_count,
                parent_node,
                )
    else:
        if current_node not in visited_for_backtrack and not graph[current_node].is_obstacle and robot.map_finished == False:
            find_direction(current_node, parent_node, robot)
            visited_for_backtrack.append(current_node)
            # print('BACK Current node: ' + current_node + ' BACK Parent node: ' + parent_node)

if __name__ == '__main__':

    graph = {
        '00': Cell('00', False, 0, Neighbor('10', '01', None, None)),
        '10': Cell('10', False, 0, Neighbor('20', '11', '00', None)),
        '20': Cell('20', False, 0, Neighbor('30', '21', '10', None)),
        '30': Cell('30', True, 0, Neighbor(None, '31', '20', None)),
        '01': Cell('01', False, 0, Neighbor('11', '02', None, '00')),
        '11': Cell('11', False, 0, Neighbor('21', '12', '01', '10')),
        '21': Cell('21', False, 0, Neighbor('31', '22', '11', '20')),
        '31': Cell('31', False, 0, Neighbor(None, '32', '21', '30')),
        '02': Cell('02', False, 0, Neighbor('12', '03', None, '01')),
        '12': Cell('12', False, 0, Neighbor('22', '13', '02', '11')),
        '22': Cell('22', True, 0, Neighbor('32', '23', '12', '21')),
        '32': Cell('32', False, 0, Neighbor(None, '33', '22', '31')),
        '03': Cell('03', False, 0, Neighbor('13', None, None, '02')),
        '13': Cell('13', False, 0, Neighbor('23', None, '03', '12')),
        '23': Cell('23', False, 0, Neighbor('33', None, '13', '22')),
        '33': Cell('33', True, 0, Neighbor(None, None, '23', '32')),
        }
    
    total_reward_count = int(sys.argv[1])
    setted_reward_count = 0
    
    while setted_reward_count < 3:
        random_cell = random.choice(list(graph.keys()))
        if graph[random_cell].is_obstacle == False and random_cell != '00':
            graph[random_cell].reward = 1
            setted_reward_count += 1 
        #graph[random.choice(list(graph.keys()))].reward = 1
    

    print ('Atil Kurt')
    visited = []
    visited_for_backtrack = []

    robot = Robot()

    robot.move_forward(rospy, robot.vel_publisher, 0.2, 0.6)    # setting robot to the -6,-6 position. Also rotations do not work properly if the robot's first move is rotation.
    collect_rewards_with_dfs(graph, '00', visited, visited_for_backtrack, robot, total_reward_count)


    print ('Goal reached')