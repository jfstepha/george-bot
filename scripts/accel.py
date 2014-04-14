#!/usr/bin/env python

import rospy
import os
import sys
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16

##################################################################################################################
##################################################################################################################
class Accel():
    ''' handles reading the accelerometers and publishing messages  '''
##################################################################################################################
##################################################################################################################
    def __init__(self):
        print "in init"
        rospy.init_node("accel")
        rospy.loginfo("accel started")
        self.rate=rospy.get_param('~rate', 10)
        self.xyzPub = rospy.Publisher('accel_xyz_raw', Int16MultiArray)
        self.scaledPub = rospy.Publisher('accel_xyz_scaled', Float32MultiArray)
        self.minPub = rospy.Publisher('accel_xyz_min', Int16MultiArray)
        self.maxPub = rospy.Publisher('accel_xyz_max', Int16MultiArray)
        self.xmin = 2048
        self.xmax = 0
        self.ymin = 2048
        self.ymax = 0 
        self.zmin = 2048
        self.zmax = 0
        
        self.sxmin = 14
        self.symin = 578
        self.szmin = 589
        
        self.sxmax = 1797 
        self.symax = 1688
        self.szmax = 1881
        

    def read_one_accel(self, n):
        f = open( "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw" % n , "ro" )
        x = f.read()
        f.close()
        return int(x)
    
    def scale(self, x, xmin, xmax):
        range = 1.0 * (xmax - xmin)
        scaled = 1.0 * ( x - xmin ) / range - 0.5
        return scaled
        

    def spin(self):
        print "spinning"
        r = rospy.Rate(self.rate)
        while not(rospy.is_shutdown()):
            # this ordering is arb
            x = self.read_one_accel(1)
            y = self.read_one_accel(2)
            z = self.read_one_accel(0)
            
            if x < self.xmin:  self.xmin = x
            if x > self.xmax:  self.xmax = x
            if y < self.ymin:  self.ymin = y
            if y > self.ymax:  self.ymax = y
            if z < self.zmin:  self.zmin = z
            if z > self.zmax:  self.zmax = z

            xs = 0 - self.scale(x, self.sxmin, self.sxmax)
            ys = self.scale(y, self.symin, self.symax)
            zs = self.scale(z, self.szmin, self.szmax)
            
            self.xyzPub.publish(data=(x,y,z))
            self.minPub.publish(data = ( self.xmin, self.ymin, self.zmin ))
            self.maxPub.publish(data = ( self.xmax, self.ymax, self.zmax ))
            self.scaledPub.publish(data = ( xs, ys, zs ))
            print "raw: %4d %4d %4d min: %4d %4d %4d max: %4d %4d %4d scaled: %5.2f %5.2f %5.2f" % (x,y,z, self.xmin, self.ymin, self.zmin, self.xmax, self.ymax, self.zmax, xs, ys, zs)
            r.sleep()

    def shutdown(self):
        print "shutting down"


#################################################################################################################
# main
##################################################################################################################
if __name__ == "__main__":
    try:
        a = Accel()
        a.spin()
    except rospy.ROSInterruptException: 
        a.shutdown()
        pass

