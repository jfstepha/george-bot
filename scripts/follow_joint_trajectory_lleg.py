#! /usr/bin/env python

import roslib; roslib.load_manifest('george')
import rospy
import actionlib
import follow_joint_trajectory

if __name__ == '__main__':
  rospy.init_node('robot_action_lleg')
  server = follow_joint_trajectory.FollowJointAction(rospy.get_name())
  rospy.spin()
