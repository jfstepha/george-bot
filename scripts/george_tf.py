#!/usr/bin/env python  
import roslib
import rospy

import tf
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Float32MultiArray
class Accel():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
    def set_xyz(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

class GeorgePoser():
    
    def __init__(self):
        rospy.init_node('george_tf_broadcaster')
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber('/accel_xyz_scaled', Float32MultiArray, self.accel_cb)
        self.accel = Accel()

    def spin(self):
        r = rospy.Rate(100)
        while( not( rospy.is_shutdown() ) ):
            self.br.sendTransform((0, 0, 0.25),  # translation
                     #tf.transformations.quaternion_from_euler(self.accel.x, self.accel.y, self.accel.z),   # rotation
                     (self.accel.x,self.accel.y,self.accel.z,1 ),   # rotation
                     rospy.Time.now(),
                     "base_link", "odom_combined")
            print ("%0.3f %0.3f %0.3f" % (self.accel.x, self.accel.y, self.accel.z))
            r.sleep()

    def accel_cb(self,msg):
        print "cb: msg: %s" % str(msg)
        self.accel.set_xyz(msg.data[0], msg.data[1], msg.data[2])

if __name__ == '__main__':
    try:
        p = GeorgePoser()
        p.spin()
    except rospy.ROSInterruptException: pass 
    #rospy.Subscriber('/%s/pose' % turtlename,
    #                 turtlesim.msg.Pose,
    #                 handle_turtle_pose,
    #                 turtlename)
    rospy.spin()