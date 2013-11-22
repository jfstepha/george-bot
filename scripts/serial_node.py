#!/usr/bin/env python

import rospy
import os
import sys
from george.msg import Appendage_state
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from robotDescription import *

path = os.path.dirname(__file__) + "/../src/"
sys.path.append(path)

DOSERVO = False

if DOSERVO:
    from Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
import time

##################################################################################################################
##################################################################################################################
class msgHandler():
##################################################################################################################
##################################################################################################################
    def __init__(self, appendage_no, pwm, pwm2 ):
        self.robot_description = RobotDescription()
        self.robot_description.ReadParameters()
        self.an = appendage_no
        if DOSERVO:
            self.pwm = pwm
            self.pwm2 = pwm2
        self.jsPub = rospy.Publisher( "joint_states" + str(self.an), JointState )
        print "msgHandler got robot description: %s " % str(self.robot_description)
        self.positions = [0.5] * self.robot_description.appendages[self.an].nservos

##################################################################################################################
    def cmd_callback(self, val):
##################################################################################################################
        rospy.logdebug( "-D- in servoHandler servo_callback appendage_no %d val:%s" % (self.an, str(val) ) )
        # import pdb; pdb.set_trace()
        js = JointState() 
        njoints = self.robot_description.appendages[self.an].nservos
        for i in range( njoints ):
            self.setServo(self.robot_description.appendages[self.an].firstservo + i, val.joints[i] )
            js.name.append( self.robot_description.appendages[self.an].jointnames[i])
            # print "val.joints = %d" % val.joints[i]
            js.position.append( val.joints[i] * 1.0)
            #self.positions[i] = ( val.joints[i] - 90)  * 0.0174532925
            self.positions[i] = val.joints[i] 
            js.velocity.append( 0.0 )
            js.effort.append( 0.0 )
            
        self.jsPub.publish( js )
        #print "-D- servoHandler servo_callback DONE appendage_no %d val:%s" % (self.an, str(val) )

##################################################################################################################
    def setServo(self,channel, angle):
##################################################################################################################
        pulse = angle * (self.servoMax - self.servoMin) / 180 + self.servoMin 
        # print "Angle = %d, setting pwm to %0.3f on channel %d" % (angle, pulse, channel)

        if DOSERVO:
            if channel < 16:
                self.pwm.setPWM(channel, 0, pulse)
            else:
                self.pwm2.setPWM(channel - 16,0, pulse)



##################################################################################################################
##################################################################################################################
class messageHandler():
##################################################################################################################
##################################################################################################################
    def __init__(self):
        self.robot_description = RobotDescription()
        self.robot_description.ReadParameters()
        self.ticksSinceUpdate_js = 0
        self.command_sub = []
        self.command_msg = []
        self.servo_handlers = []
        self.state_pub = []
        self.state_msg = []
        self.debug_sub = []
        self.jpc_pub = []
        rospy.init_node('jointstate_to_pi')
        
        ### set up command message subscriptions ########
        self.prev_msg = [] * self.robot_description.NAppendages
        self.servo_setup()
        k=0
        for i in range( self.robot_description.NAppendages ):
            njoints = len( self.robot_description.appendages[i].jointnames )
            prev_msg_tmp = [255] * njoints
            self.prev_msg.append(prev_msg_tmp)
            rospy.loginfo("-D- jointstate_to_pi setting up appendage #%d" % i)
            s = msgHandler( i, self.pwm, self.pwm2 )
            s.servoMax = self.servoMax
            s.servoMin = self.servoMin
            self.command_sub.append( rospy.Subscriber( "command"+str(i), Appendage_state, s.cmd_callback ) )
            self.servo_handlers.append( s )
            for j in range( self.robot_description.appendages[i].nservos):
                self.jpc_pub.append( rospy.Publisher("/george/joint%d_position_controller/command" % k, Float64 ))
                k=k+1
                
            
        self.full_js_pub = rospy.Publisher("joint_states", JointState)    

##################################################################################################################
    def servo_setup(self):
##################################################################################################################
        if DOSERVO:
            self.pwm = PWM(0x40, debug=True)
            self.pwm2 = PWM(0x41, debug=True)
            self.pwm.setPWMFreq(60)
            self.pwm2.setPWMFreq(60)
        else:
            self.pwm = False
            self.pwm2 = False
        #self.setServoPulse( 0, 0)

        self.servoMin = 150  # Min pulse length out of 4096
        self.servoMax = 600  # Max pulse length out of 4096
        
##################################################################################################################
    def full_jointstate_publish(self):
##################################################################################################################
        js = JointState() 
        k=0
        for i in range( self.robot_description.NAppendages ):
            njoints = self.robot_description.appendages[i].nservos
            for j in range( njoints ):
                js.name.append( self.robot_description.appendages[i].jointnames[j])
                # print "val.joints = %d" % val.joints[i]
                js.position.append( self.servo_handlers[i].positions[j] ) 
                self.jpc_pub[k].publish( self.servo_handlers[i].positions[j])
                k=k+1
            js.velocity.append( 0.0 )
            js.effort.append( 0.0 )
            js.header.frame_id = "base_link"
            js.header.stamp = rospy.Time.now()
            
        self.full_js_pub.publish( js )
            
##################################################################################################################
    def spin(self):
##################################################################################################################
        while not(rospy.is_shutdown()):
            rospy.sleep(0.05)
            self.full_jointstate_publish();
            
##################################################################################################################
# main
##################################################################################################################
if __name__ == "__main__":
    try:
        mh = messageHandler()
        mh.spin()
    except rospy.ROSInterruptException: pass 

