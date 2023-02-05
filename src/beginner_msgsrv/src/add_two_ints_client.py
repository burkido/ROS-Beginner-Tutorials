#!/usr/bin/env python

#Make the node an executable node
#chmod u+x ~/catkin_ws/src/beginner_msgsrv/src/add_two_ints_client.py 

from __future__ import print_function

import sys
import rospy
from beginner_msgsrv.srv import *

def add_two_ints_client(x, y):

    #This method blocks until the service named add_two_ints is available.
    rospy.wait_for_service('add_two_ints')

    try:
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)

	#Because we've declared the type of the service to be AddTwoInts, 
	#it does the work of generating the AddTwoIntsRequest object for you.
	# 
	#The return value is an AddTwoIntsResponse object.
	#
	#If the call fails, a rospy.ServiceException may be thrown, 
	#so you should setup the appropriate try/except block.
        resp1 = add_two_ints(x, y)
        return resp1.sum
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print(usage())
        sys.exit(1)
    print("Requesting %s+%s"%(x, y))
    print("%s + %s = %s"%(x, y, add_two_ints_client(x, y)))
