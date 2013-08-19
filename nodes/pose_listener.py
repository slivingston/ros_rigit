#!/usr/bin/env python
"""
SCL; 14 Aug 2013
"""
import roslib; roslib.load_manifest('ros_rigit')
import rospy
import tf
from ros_rigit.msg import pose_objects

import numpy
from numpy import *

class Listener:
    def __init__(self):
        rospy.init_node('pose_listener', anonymous=True)
        rospy.Subscriber('pose_estimation', pose_objects, self.callback)
        self.br = tf.TransformBroadcaster()
        rospy.spin()
        
    def callback(self, pose_packet):
        for p in pose_packet.bodies:
            R = numpy.zeros((4,4))
            R[3,3] = 1.
            R[:3,:3] = array(p.R).reshape(3,3)
            T = array(p.T)
            print '-'*80
            print p.name
            print 'R = '
            print R
            print 'T = '
            print T
            self.br.sendTransform(T,
                                  tf.transformations.quaternion_from_matrix(R),
                                  rospy.Time.now(), p.name+str("/odom"), "/odom")

if __name__ == '__main__':
    listener = Listener()
