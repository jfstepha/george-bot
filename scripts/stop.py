#!/usr/bin/env python

from george.srv import *
import rospy

def handle_stop(req):
    print "-D- in stop service handler"
    return 0

def stop_server():
    rospy.init_node('stop_server')
    s = rospy.Service('stop', Stop, handle_stop)
    print "Ready to stop"
    rospy.spin()

if __name__ == "__main__":
    stop_server()