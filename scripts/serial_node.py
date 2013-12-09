#!/usr/bin/env python

import rospy
import os
import sys
from george.msg import Appendage_state
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import String
import yaml

from robotDescription import *

path = os.path.dirname(__file__) + "/../src/"
sys.path.append(path)


import time

##################################################################################################################
##################################################################################################################
class msgHandler():
    ''' handles the messages for each appendage '''
##################################################################################################################
##################################################################################################################
    def __init__(self, appendage_no, pwm, pwm2, doservo ):
        self.robot_description = RobotDescription()
        self.robot_description.ReadParameters()
        self.an = appendage_no
        self.DOSERVO = doservo
        if self.DOSERVO:
            self.pwm = pwm
            self.pwm2 = pwm2
        self.jsPub = rospy.Publisher( "joint_states" + str(self.an), JointState )
        print "msgHandler got robot description: %s " % str(self.robot_description)
        self.positions = [0.5] * self.robot_description.appendages[self.an].nservos
        self.trims = [0] * self.robot_description.appendages[self.an].nservos

##################################################################################################################
    def cmd_callback(self, val):
##################################################################################################################
        rospy.logdebug( "-D- in servoHandler servo_callback appendage_no %d val:%s" % (self.an, str(val) ) )
        # import pdb; pdb.set_trace()
        js = JointState() 
        njoints = self.robot_description.appendages[self.an].nservos
        for i in range( njoints ):
            self.setServo(self.robot_description.appendages[self.an].firstservo + i, val.joints[i] + self.trims[i])
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
        pulse = int( (angle / 3.1416 + 0.5) * (self.servoMax - self.servoMin) + self.servoMin )
        rospy.logdebug("Angle = %0.3f, setting pwm to %0.3f on channel %d" % (angle, pulse, channel))

        if self.DOSERVO:
            rospy.loginfo("serial_node setServo setting channel %d to pulse %d" %(channel, angle))
            if channel < 16:
                self.pwm.setPWM(channel, 0, pulse)
            else:
                self.pwm2.setPWM(channel - 16,0, pulse)
        else:
            rospy.loginfo("serial_node setServo DOSERVO not set, param from launch: DOSERVO: %s" % (str(self.DOSERVO) ) )
##################################################################################################################
    def print_trim(self):
##################################################################################################################
        rospy.loginfo("Trim for appendage #%d: %s" % (self.an, str(self.trims)))

##################################################################################################################
    def get_trim(self):
##################################################################################################################
        rospy.loginfo("Getting trim for appendage #%d" % self.an ) 
        njoints = self.robot_description.appendages[self.an].nservos

        trim_dict = {}    
        for i in range( njoints ):
            j_name = self.robot_description.appendages[self.an].jointnames[i]
            trim_dict[ j_name ] = self.trims[i]
            
        return trim_dict

##################################################################################################################
    def set_trim(self):
##################################################################################################################
        rospy.loginfo("Setting trim for appendage #%d" % self.an ) 
        njoints = self.robot_description.appendages[self.an].nservos
    
        for i in range( njoints ):
            newtrim = 0 - self.positions[i] + self.trims[i]
            rospy.logdebug("serial_node set_trim an: %d joint %d joint value: %0.3f prev_trim: %0.3f new_trim %0.3f" % 
                          (self.an, i, self.positions[i], self.trims[i], newtrim) )
            self.trims[i] = newtrim

##################################################################################################################
    def set_trim_from_dict(self, trim_dict):
##################################################################################################################
        rospy.loginfo("Setting trim for appendage #%d to %s " % (self.an, str(trim_dict) ) )
        njoints = self.robot_description.appendages[self.an].nservos
    
        for i in range( njoints ):
            newtrim = trim_dict[ self.robot_description.appendages[self.an].jointnames[i] ]
            rospy.logdebug("serial_node set_trim an: %d joint %d joint value: %0.3f prev_trim: %0.3f new_trim %0.3f" % 
                          (self.an, i, self.positions[i], self.trims[i], newtrim) )
            self.trims[i] = newtrim






##################################################################################################################
##################################################################################################################
class messageHandler():
    ''' handles the messages for the whole robot '''
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
        
        self.DOSERVO = rospy.get_param('do_servos','false')

        if self.DOSERVO:
            from Adafruit_PWM_Servo_Driver.Adafruit_PWM_Servo_Driver import PWM
        
        ### set up command message subscriptions ########
        self.prev_msg = [] * self.robot_description.NAppendages
        if self.DOSERVO:
            self.servo_setup(PWM)
        else:
            self.servo_setup(False)
        k=0
        for i in range( self.robot_description.NAppendages ):
            njoints = len( self.robot_description.appendages[i].jointnames )
            prev_msg_tmp = [255] * njoints
            self.prev_msg.append(prev_msg_tmp)
            rospy.loginfo("-D- jointstate_to_pi setting up appendage #%d" % i)
            s = msgHandler( i, self.pwm, self.pwm2, self.DOSERVO )
            s.servoMax = self.servoMax
            s.servoMin = self.servoMin
            self.command_sub.append( rospy.Subscriber( "command"+str(i), Appendage_state, s.cmd_callback ) )
            self.servo_handlers.append( s )
            for j in range( self.robot_description.appendages[i].nservos):
                self.jpc_pub.append( rospy.Publisher("/george/joint%d_position_controller/command" % k, Float64 ))
                k=k+1
            
        self.full_js_pub = rospy.Publisher("joint_states", JointState)    
        self.macro_sub = rospy.Subscriber( "macro_cmd", String, self.macro_callback)

##################################################################################################################
    def servo_setup(self, PWM):
##################################################################################################################
        if self.DOSERVO:
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
    def macro_callback(self, msg):
##################################################################################################################
        rospy.logdebug("-D- serial_node received macro command: %s" % msg.data)
        if msg.data == "print_trim":
            self.print_trim()
        elif msg.data == "set_trim":
            self.set_trim()
        elif msg.data == "save_trim":
            self.save_trim()
        elif msg.data == "load_trim":
            self.load_trim()
            
            
##################################################################################################################
    def print_trim(self):
##################################################################################################################
        for i in range( self.robot_description.NAppendages ):
            self.servo_handlers[i].print_trim()

##################################################################################################################
    def set_trim(self):
##################################################################################################################
        for i in range( self.robot_description.NAppendages ):
            self.servo_handlers[i].set_trim()
            
##################################################################################################################
    def save_trim(self):
##################################################################################################################
        rospy.loginfo("serial_node saving trim:")
        trim_dict = {}
        for i in range( self.robot_description.NAppendages ):
            a_name = self.robot_description.appendages[i].name
            trim_dict[ a_name ] = self.servo_handlers[i].get_trim()
        rospy.loginfo("trim dict: %s" % str(trim_dict))
        rospy.loginfo("saving trims to ~/.ros/george_trim.yaml")
        stream = file('george_trim.yaml', 'w')    # 'document.yaml' contains a single YAML document.
        yaml.dump( trim_dict, stream, default_flow_style=False )
        stream.close()

##################################################################################################################
    def load_trim(self):
##################################################################################################################
        rospy.loginfo("serial_node loading trim:")
        rospy.loginfo("loading from ~/.ros/george_trim.yaml")
        stream = file('george_trim.yaml', 'r')    # 'document.yaml' contains a single YAML document.
        trim_dict = yaml.load( stream )
        stream.close()
        rospy.loginfo ("got: %s" % str(trim_dict))

        for i in range( self.robot_description.NAppendages ):
            rospy.logdebug( "Looking up appendage #%d name %s" % ( i, self.robot_description.appendages[i].name))
            self.servo_handlers[i].set_trim_from_dict( trim_dict[ self.robot_description.appendages[i].name ] )
        

        
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

