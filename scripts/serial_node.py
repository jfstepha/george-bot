#!/usr/bin/env python

import rospy
from george.msg import Appendage_state
from sensor_msgs.msg import JointState

from robotDescription import *

##################################################################################################################
##################################################################################################################
class msgHandler():
##################################################################################################################
##################################################################################################################
    def __init__(self, appendage_no ):
        self.robot_description = RobotDescription()
        self.robot_description.ReadParameters()
        self.an = appendage_no
        self.jsPub = rospy.Publisher( "joint_states" + str(self.an), JointState )
        print "msgHandler got robot description: %s " % str(self.robot_description)
    def cmd_callback(self, val):
        print "-D- in servoHandler servo_callback appendage_no %d val:%s" % (self.an, str(val) )
        # import pdb; pdb.set_trace()
        js = JointState() 
        njoints = self.robot_description.appendages[self.an].nservos
        for i in range( njoints ):
            js.name.append( self.robot_description.appendages[self.an].jointnames[i])
            print "val.joints = %d" % val.joints[i]
            js.position.append( val.joints[i] * 1.0)
            js.velocity.append( 0.0 )
            js.effort.append( 0.0 )
            
        self.jsPub.publish( js )

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
        rospy.init_node('jointstate_to_pi')
        
        ### set up command message subscriptions ########
        self.prev_msg = [] * self.robot_description.NAppendages
        for i in range( self.robot_description.NAppendages ):
            njoints = len( self.robot_description.appendages[i].jointnames )
            prev_msg_tmp = [255] * njoints
            self.prev_msg.append(prev_msg_tmp)
            rospy.loginfo("-D- jointstate_to_pi setting up appendage #%d" % i)
            s = msgHandler( i )
            self.command_sub.append( rospy.Subscriber( "command"+str(i), Appendage_state, s.cmd_callback ) )
            self.servo_handlers.append( s )
            
    def spin(self):
        while not(rospy.is_shutdown()):
            rospy.sleep(0.01)
            
##################################################################################################################
# main
##################################################################################################################
if __name__ == "__main__":
    try:
        mh = messageHandler()
        mh.spin()
    except rospy.ROSInterruptException: pass 

