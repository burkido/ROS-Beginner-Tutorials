#!/usr/bin/env python

#Make the node an executable node
#chmod u+x ~/catkin_ws/src/beginner_msgsrv/src/add_two_ints_server.py 

from __future__ import print_function

from beginner_msgsrv.srv import AddTwoInts, AddTwoIntsResponse
import rospy

def handle_add_two_ints(req):
    print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    
    #This declares a new service named add_two_ints with the AddTwoInts service type. 
    #All requests are passed to handle_add_two_ints function. 
    #handle_add_two_ints is called with instances of AddTwoIntsRequest and returns instances of AddTwoIntsResponse.
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    print("Ready to add two ints.")

    #rospy.spin() keeps your code from exiting until the service is shutdown.
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()