import rospy
from robotDescription import *
robot_description = RobotDescription()
from moveit_commander import MoveGroupCommander 

##########################################################################
##########################################################################
class BipedCommander():
##########################################################################
##########################################################################
    def __init__(self):
            rospy.loginfo("BipedCommander started")
            
    def move_lfoot(self, x, y, z):
        rospy.loginfo("BipedCommander move_lfoot")