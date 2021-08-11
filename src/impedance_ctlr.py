#!/usr/bin/env python

# Hannah Kolano kolanoh@oregonstate.edu
#
# Proxy for pruning project: subscribes to force data from UR5 
#
# Last modified 8/11/2021 by Hannah

import rospy
import sys
from numpy import sin, cos, pi 

from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Int64

class AdmitCtlr():

    def __init__(self):
        '''
        Set up subscriber to the force torque sensor
        '''

        # Subscribe to wrist torque topic
        self.wrench_sub = rospy.Subscriber('wrench_measurement', Wrench, self.wrench_callback)
        rospy.loginfo("Force subscriber node initialized.")

    def wrench_callback(self, wrench_msg):
        """
        Callback function to deal with incoming wrench messages
        """
        rospy.loginfo("Force subscriber received a wrench message!")

        # Get information about the message
        # forces = wrench_msg.force 
        # moments = wrench_msg.torque
        rospy.loginfo('Got {0}'.format(wrench_msg.force.x))

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('imp_ctlr', argv=sys.argv)

    ctlr = AdmitCtlr()
    
    rospy.spin()