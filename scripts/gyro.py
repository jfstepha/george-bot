#!/usr/bin/env python

import rospy
import os
import sys
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Int16

path = os.path.dirname(__file__) + "/../src/"
sys.path.append(path)

##################################################################################################################
##################################################################################################################
class GyroMsgs():
    ''' handles reading the gyros and publishing messages  '''
##################################################################################################################
##################################################################################################################
    def __init__(self ):
        print "in init"
        rospy.init_node("gyro")
        from Gyro.Gyro import Gyro
        self.gyro = Gyro( 0x6b )
        rospy.loginfo("GyroMsg started")
        rospy.loginfo("gyro whoami: 0x%x " % self.gyro.whoami() )
        rospy.loginfo("gyro setting up config registers... " )
        self.gyro.setupCRs()
        rospy.loginfo("gyro calibrating... " )
        self.gyro.calibrate()
        self.gyroPub = rospy.Publisher( "gyros", Float32MultiArray)
        self.gyroSpinsPub =  rospy.Publisher( "gyros_spins", Int16)
        self.rate = rospy.get_param('~rate', 1)
        

##################################################################################################################
    def spin(self):
        r = rospy.Rate(self.rate)
        while not(rospy.is_shutdown()):
            #print "spinning"
            self.gyro.multispin(40)

            xh = self.gyro.xh
            yh = self.gyro.yh
            zh = self.gyro.zh

            x = self.gyro.x
            y = self.gyro.y
            z = self.gyro.z

            xt = self.gyro.xt
            yt = self.gyro.yt
            zt = self.gyro.zt
            self.gyroPub.publish(data=(xt,yt,zt))
            self.gyroSpinsPub.publish(data=self.gyro.spins)
            r.sleep()
            #rospy.loginfo( " xyz: %8.3f %8.3f %8.3f  " % ( x,y,z ) + "tot: %11.6f, %11.6f, %11.6f" % (xt, yt, zt) + " s:%d" % self.gyro.spins )


##################################################################################################################
# main
##################################################################################################################
if __name__ == "__main__":
    try:
        gm = GyroMsgs()
        gm.spin()
    except rospy.ROSInterruptException: pass 

