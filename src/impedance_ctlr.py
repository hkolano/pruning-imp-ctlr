#!/usr/bin/env python

# Hannah Kolano kolanoh@oregonstate.edu
#
# Proxy for pruning project: subscribes to force data from UR5 
#
# Last modified 8/11/2021 by Hannah

import rospy
import sys
import numpy as np

from geometry_msgs.msg import Wrench, Vector3Stamped, Vector3

class AdmitCtlr():

    def __init__(self):
        '''
        Set up subscriber to the force torque sensor
        '''

        # Subscribe to wrist torque topic
        self.wrench_sub = rospy.Subscriber('wrench_measurement', Wrench, self.wrench_callback)
        # Publish velocities to robot
        self.vel_pub = rospy.Publisher('/vel_command', Vector3Stamped, queue_size=5)
        rospy.loginfo("Force subscriber node initialized.")

        # Set up parameters
        # Desired (reference) wrench
        self.des_wrench = np.array([0, 0, 0, 0, 0, -0.02])
        # Controller gain 
        self.Kf = .5
        # Selection matrix
        self.l = np.diag([1, 0, 0, 0, 1, 1])

        self.vel = Vector3Stamped()
        self.vel.header.stamp = rospy.Time.now()
        # self.vel.frameid = tool_frame
        self.vel.vector = Vector3(0.0, 0.0, 0.0)


    def wrench_callback(self, wrench_msg):
        """
        Callback function to deal with incoming wrench messages
        """
        rospy.loginfo("Force subscriber received a wrench message!")

        # Write the wrench_msg into an array
        wrench = np.array([wrench_msg.torque.x, 0, 0, 0, wrench_msg.force.y, wrench_msg.force.z])
        rospy.loginfo('Got wrench {0}'.format(wrench))
        # idealized velocities
        d_xdes = -self.Kf*np.dot(self.l,self.des_wrench-wrench)
        rospy.loginfo(d_xdes.shape)

        # Set up the velocity command
        self.vel.header.stamp = rospy.Time.now()
        # rospy.loginfo(d_xdes[5])
        self.vel.vector = Vector3(0.0, d_xdes[4], d_xdes[5])
        self.vel_pub.publish(self.vel)

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('imp_ctlr', argv=sys.argv)

    ctlr = AdmitCtlr()

    rospy.spin()