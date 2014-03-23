#! /usr/bin/env python

import roslib; roslib.load_manifest('george')
import rospy
import time
import actionlib
from sensor_msgs.msg import JointState
from george.msg import Appendage_state

import george.msg
###################################################################### 
###################################################################### 
class FollowJointAction():
###################################################################### 
###################################################################### 
    #_feedback = biped_controls.msg.FollowJointTrajectoryFeedback()
    #_result   = biped_controls.msg.FollowJointTrajectoryResult()
    command_msg = Appendage_state();

###################################################################### 
    def __init__(self, name):
###################################################################### 
        self._action_name = name
        # rospy.loginfo("FollowJointAction %s started" % name)
        self._as = actionlib.SimpleActionServer('follow_joint_trajectory', george.msg.FollowJointTrajectoryAction, self.execute, False)
        self._as.start()

        self.command_pub = rospy.Publisher("joint_cmd", Appendage_state)
        self.action_sub = rospy.Subscriber("joint_states", JointState, self.jointstate_callback, None, 1)
        self.robot_state = []
        self.robot_state_name = []
        #  robot_state[robot_state_index[i]] is the same joint as joint_state[i]
        self.robot_state_index = []
        self.f = open("~/actionlog" + name, 'w')
    

###################################################################### 
    def execute(self, goal):
###################################################################### 
    # helper variables
        # Override for debug:
        self.f.write(goal)

        r = rospy.Rate(1000)
        success = True
        rospy.logdebug("-D- robot_action in execute_cb")
        # publish info to the console for the user
        # rospy.loginfo('%s: Executing trajectory' % self._action_name)
        
        # start executing the action
        if self._as.is_preempt_requested():
            # rospy.loginfo('%s: Preempted' % self._action_name)
            self._as.set_preempted()
            success = False
        else:
            #self.command_msg.header.stamp = rospy.Time.now()
            #self.command_msg.name = goal.trajectory.joint_names
            #self._feedback.joint_names = goal.trajectory.joint_names
            
            self.set_jointstate_lookup(goal.trajectory.joint_names) 
            # rospy.loginfo("robot_action looping over %d points" % len(goal.trajectory.points))
            start_time = rospy.Time.now()

            for i in range(len(goal.trajectory.points)):
                t = goal.trajectory.points[i].time_from_start
                # rospy.loginfo("setting joints to %s, t: %s, time since start: %s" % ( str(goal.trajectory.points[i].positions), str(t), str(rospy.Time.now() - start_time)))
                

                ## check for empty arrays:
                # rospy.loginfo("lengths: command_msg.joints: %d robot_state_index: %d positions: %d" % (
                #            len( self.command_msg.joints ), len( self.robot_state_index), len( goal.trajectory.points[i].positions)))
                if len( self.robot_state_index )== 0:
                    rospy.logwarn( "robot_state_index is empty" )
                    rospy.sleep(0.001)
                    continue
                if len( self.command_msg.joints )== 0:
                    rospy.logwarning( "command_msg is empty" )
                    rospy.sleep(0.001)
                    continue

                for j in range(len(goal.trajectory.points[i].positions)):
                    self.command_msg.joints[ self.robot_state_index[j] ] = goal.trajectory.points[i].positions[j] 
                # rospy.logdebug("set command_msg.joints to %s" % str( self.command_msg.joints ))

                self.command_pub.publish(self.command_msg)
                # rospy.logdebug("robot_action waiting to reach goal #%d" % i)
                success = self.waitToReachGoal(self.command_msg)

                # wait for arrival time to make sure it's timed right
                while rospy.Time.now() - start_time < t:
                    rospy.sleep(0.025)

           #     if success:
           #          rospy.loginfo("robot_action reached goal")
                    #self._feedback.actual.positions = self.robot_state
                    #self._as.publish_feedback(self._feedback)
           #     if not success:
           #         rospy.logerr("%s: error: did not reach goal" % self._action_name)
                    #self._as.set_aborted(self._result)
           #         self._as.set_aborted()
                    return 
                # r.sleep()
            
        # publish the feedback
        #self._as.publish_feedback(self._feedback)
          
        if success:
          # rospy.loginfo('%s: Succeeded' % self._action_name)
          #self._as.set_succeeded(self._result)
          self._as.set_succeeded()

            
        
###################################################################### 
    def jointstate_callback(self,msg):
###################################################################### 
        # rospy.loginfo('robot_action received message from robot')
        try:
            for i in range(len(msg.name)):
                self.robot_state[i] = msg.position[i]
                self.robot_state_name[i] = msg.name[i]
            
        except:
            self.robot_state = []
            self.robot_state_name = []
            for i in range(len(msg.name)):
                self.robot_state.append(msg.position[i])
                self.robot_state_name.append(msg.name[i])
                
        # rospy.loginfo("set robot_state to %s" % (self.robot_state))
    ######################################################################
    def set_jointstate_lookup(self, joint_names):
    ######################################################################
        # rospy.loginfo("robot_action - set_jointstate_lookup jointnames: %s in %s" % (str(joint_names), str(self.robot_state_name) ) )
        #if len( self.robot_state_index ) == 0:
        #    rospy.loginfo("robot state index not set yet")
        #    return()
        if len( joint_names ) == 0:
            rospy.loginfo("joint names not set yet")
            return()
        self.robot_state_index = []
        for i in range(len(joint_names)):
            self.robot_state_index.append( self.robot_state_name.index(joint_names[i]) )
            # rospy.loginfo("robot_action - set_jointstate_lookup robot_state_inded: %s" % str(self.robot_state_index))
            
    ######################################################################
    def waitToReachGoal(self, msg):
    ######################################################################
      start_time = rospy.Time.now()
      
      any_mismatch = True
      elapsed_time = 0
      while ((elapsed_time < 3.0) & (any_mismatch)):
          # rospy.loginfo("  waiting...")
          
          any_mismatch = False
          # rospy.logdebug("comparing joints %s to robot_state %s" % (msg.joints, self.robot_state))
          if len( self.robot_state) < len( msg.joints):
              #rospy.logdebug("robot_state is empty")
              rospy.sleep(0.001)
              continue
          for i in range(len(msg.joints)):
#              try:
                  any_mismatch = False
                  #rospy.logdebug("joints is %d long, robot_state is %d long" % (len(msg.joints), len(self.robot_state)))
                  #rospy.logdebug("robot_action - comparing state of #%d: %0.3f vs %0.3f" % (i, msg.joints[i], self.robot_state[i]))
#                  # we  should use a tolerance passed in from the goal, but it seems to be empty
                  #f abs(msg.joints[i] - self.robot_state[self.robot_state_index[i]]) > 0.03:
                  if abs(msg.joints[i] - self.robot_state[i]) > 0.03:
                      # rospy.logdebug("robot_action setting mismatch to true")
                      any_mismatch = True
                      now_time = rospy.Time.now()
                      elapsed_time = now_time.secs - start_time.secs + (now_time.nsecs - start_time.nsecs) / 1e9
                      # rospy.logdebug("robot_action elapsed time: %0.8f" % (elapsed_time))
#              except :
#                  rospy.logerr("robot_action exception ! - on joint %s " % (msg.name[i]))
#          rospy.sleep(0.001)        
          
      if any_mismatch:
         rospy.logerr("robot_action - timed out")             
      return not(any_mismatch)    

    ######################################################################
    def spin(self):
    ######################################################################
        while not rospy.is_shutdown():
            rospy.sleep(10)
        self.f.close()
            
        
  


###################################################################### 
#if __name__ == '__main__':
###################################################################### 
#  rospy.init_node('follow_joint_trajectory')
#  rospy.spin()
    
