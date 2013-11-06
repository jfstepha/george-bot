#!/usr/bin/env python

from george.srv import *
import rospy

def handle_home(req):
    print "-D- in home service handler"
    return 0

def home_server():
    rospy.init_node('home_server')
    s = rospy.Service('home', Home, handle_home)
    print "Ready to go home"
    rospy.spin()

if __name__ == "__main__":
    home_server()