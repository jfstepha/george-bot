import rospy
from robotDescription import *
robot_description = RobotDescription()
from moveit_commander import MoveGroupCommander 

## walking parameters
x_spread = 0.045   # from center to foot
y_start = 0.0 
z_start = -0.19
        
z_squat = -0.185
step_height = 0.01
stride_len = 0.02

##########################################################################
##########################################################################
class BipedCommander():
##########################################################################
##########################################################################

    ##########################################################################
    def __init__(self):
    ##########################################################################
            rospy.loginfo("BipedCommander started")
            self.legs_group = MoveGroupCommander("legs")
            self.arms_group = MoveGroupCommander("arms")
            
    ##########################################################################
    def move_lfoot(self, x, y, z):
    ##########################################################################
        rospy.loginfo("BipedCommander move_lfoot")
        end_effector_link = "lfoot"
        self.move_foot( end_effector_link, x, y, z)

    ##########################################################################
    def move_rfoot(self, x, y, z):
    ##########################################################################
        rospy.loginfo("BipedCommander move_rfoot")
        end_effector_link = "rfoot"
        self.move_foot( end_effector_link, x, y, z)
        
    ##########################################################################
    def move_foot(self, end_effector_link, x, y, z):
    ##########################################################################
        pose = [x, y, z, 0, 0.7071, 0.7071, 0]
        if len(end_effector_link) > 0 or self.has_end_effector_link():
            rospy.logdebug("setting target")
            r = self.legs_group.set_pose_target(pose, end_effector_link)
            rospy.loginfo("set position target returned %s" % str(r)) 
            rospy.logdebug("going")
            r =  self.legs_group.go()
            rospy.loginfo("go returned %s" % str(r)) 
        else:
            rospy.logerr("There is no end effector to set the pose for")
       
    ##########################################################################
    def move_legs(self, lx, ly, lz, rx, ry, rz): 
    ##########################################################################
        pose = [lx, ly, lz, 0, 0.7071, 0.7071, 0]
        end_effector_link = "lfoot"
        rospy.loginfo("setting lfoot target to %s" % pose)
        r = self.legs_group.set_pose_target(pose, end_effector_link)
        rospy.loginfo("set lfoot position target returned %s" % str(r)) 

        pose = [rx, ry, rz, 0, 0.7071, 0.7071, 0]
        rospy.loginfo("setting rfoot target to %s" % pose)
        end_effector_link = "rfoot"
        r = self.legs_group.set_pose_target(pose, end_effector_link)
        rospy.loginfo("set rfoot position target returned %s" % str(r)) 

        r =  self.legs_group.go()
        rospy.loginfo("go returned %s" % str(r)) 
        
    ##########################################################################
    def pose_print(self):
    ##########################################################################
        end_effector_link = "lfoot"
        r = self.legs_group.get_current_pose( end_effector_link)
        p = r.pose.position
        o = r.pose.orientation
        rospy.loginfo("Left foot pose: [%0.3f, %0.3f, %0.3f], [%0.3f, %0.3f, %0.3f, %0.3f] " % (p.x, p.y, p.z, o.x, o.y, o.z, o.w))

        end_effector_link = "rfoot"
        r = self.legs_group.get_current_pose( end_effector_link)
        p = r.pose.position
        o = r.pose.orientation
        rospy.loginfo("Right foot pose: [%0.3f, %0.3f, %0.3f], [%0.3f, %0.3f, %0.3f, %0.3f] " % (p.x, p.y, p.z, o.x, o.y, o.z, o.w))

        end_effector_link = "ltip3"
        r = self.arms_group.get_current_pose( end_effector_link)
        p = r.pose.position
        o = r.pose.orientation
        rospy.loginfo("Left arm pose: [%0.3f, %0.3f, %0.3f], [%0.3f, %0.3f, %0.3f, %0.3f] " % (p.x, p.y, p.z, o.x, o.y, o.z, o.w))

        end_effector_link = "rtip3"
        r = self.arms_group.get_current_pose( end_effector_link)
        p = r.pose.position
        o = r.pose.orientation
        rospy.loginfo("Right arm pose: [%0.3f, %0.3f, %0.3f], [%0.3f, %0.3f, %0.3f, %0.3f] " % (p.x, p.y, p.z, o.x, o.y, o.z, o.w))
        
    ##########################################################################
    def walk_pose(self, r_pose, l_pose, pose_name):
    ##########################################################################
        ree = "rfoot" # right leg end effector
        lee = "lfoot" # right leg end effector

        rospy.loginfo("walk: going to %s", pose_name)
        rospy.loginfo("poses: right:%s left:%s" % (r_pose, l_pose))
        self.legs_group.set_pose_target( r_pose, ree)
        self.legs_group.set_pose_target( l_pose, lee)
        r = self.legs_group.go()
        rospy.loginfo("walk: go returned %s" % r)
        if (r == True):
            return(True)
        else:
            rospy.logerr("**walk pose failed on pose %s" % pose_name)
            return(False)

        return(True)

    ##########################################################################
    def walk_first_step(self):
    ##########################################################################
        
        # starting pose
        r_pose = [-x_spread, y_start, z_start, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread, y_start, z_start, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "start"):
            return(False)
        
        # squat a little
        r_pose = [-x_spread, y_start, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread, y_start, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "squat"):
            return(False)

        # lean to one side 
        r_pose = [-x_spread*2, y_start, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0, y_start, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "lean"):
            return(False)

        # raise up leg
        r_pose = [-x_spread*2, y_start, z_squat + step_height, 0, 0.7071, 0.7071, 0]
        l_pose = [0, y_start, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "raise_leg"):
            return(False)

        # step forward
        r_pose = [-x_spread*2, y_start - stride_len / 2, z_squat + step_height, 0, 0.7071, 0.7071, 0]
        l_pose = [0, y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step_forward"):
            return(False)
        
        # step down 
        r_pose = [-x_spread*2, y_start - stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0, y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step_down"):
            return(False)

        return(True)

    ##########################################################################
    def walk_left_step(self):
    ##########################################################################
        # starting pose
        r_pose = [-x_spread * 2, y_start - stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "start"):
            return(False)
        
        # center forward
        r_pose = [-x_spread * 2, y_start,                  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start + stride_len,     z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "center_forward"):
            return(False)

        # center center
        r_pose = [-x_spread,    y_start,                  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread,     y_start + stride_len,     z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "shift_to_other_foot"):
            return(False)

        # shift to other foot
        r_pose = [0,            y_start,                  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start + stride_len,     z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "shift_to_other_foot"):
            return(False)

        # raise rear leg
        r_pose = [0,            y_start,                 z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start + stride_len,    z_squat + step_height, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "raise_rear_leg"):
            return(False)

        # step forward
        r_pose = [0,            y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start - stride_len / 2, z_squat + step_height, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step_forward"):
            return(False)

        # step down
        r_pose = [0,            y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start - stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step_forward"):
            return(False)
        
        return(True)

    ##########################################################################
    def walk_right_step(self):
    ##########################################################################

        # starting pose
        r_pose = [0,            y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start - stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "start"):
            return(False)

        # center forward
        r_pose = [0,            y_start + stride_len,   z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start               , z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "center forward"):
            return(False)

        # center center
        r_pose = [-x_spread,   y_start + stride_len,  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread,    y_start             ,  z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "center center"):
            return(False)

        # shift to other foot
        r_pose = [-x_spread * 2, y_start + stride_len, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start             , z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "shift_to_other_foot"):
            return(False)

        # raise rear leg
        r_pose = [-x_spread * 2, y_start + stride_len, z_squat + step_height, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start,              z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "raise rear leg"):
            return(False)

        # step forward
        r_pose = [-x_spread * 2, y_start - stride_len / 2, z_squat + step_height, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step forward"):
            return(False)

        # step down
        r_pose = [-x_spread * 2, y_start - stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step down"):
            return(False)
    
        return(True)

    ##########################################################################
    def walk_right2home(self):
    ##########################################################################

        # starting pose
        r_pose = [-x_spread * 2, y_start - stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start + stride_len / 2, z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "start"):
            return(False)

        # center forward
        r_pose = [-x_spread * 2, y_start,                  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [0,             y_start + stride_len,     z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "center_forward"):
            return(False)

        # center center
        r_pose = [-x_spread,    y_start,                  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread,     y_start + stride_len,     z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "shift_to_other_foot"):
            return(False)

        # shift to other foot
        r_pose = [0,            y_start,                  z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start + stride_len,     z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "shift_to_other_foot"):
            return(False)

        # raise rear leg
        r_pose = [0,            y_start,                 z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start + stride_len,    z_squat + step_height, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "raise_rear_leg"):
            return(False)

        # step forward
        r_pose = [0,            y_start                 , z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start                 , z_squat + step_height, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "step_forward"):
            return(False)

        # foot down
        r_pose = [0,            y_start                 , z_squat, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread * 2, y_start                 , z_squat, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "foot down"):
            return(False)

        # home
        r_pose = [-x_spread,            y_start                 , z_start, 0, 0.7071, 0.7071, 0]
        l_pose = [x_spread,             y_start                 , z_start, 0, 0.7071, 0.7071, 0]
        if not self.walk_pose(r_pose, l_pose, "home"):
            return(False)
        
        return(True)