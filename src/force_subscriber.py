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

def wrench_callback(wrench_msg):
    """
    Callback function to deal with incoming wrench messages
    """
    rospy.loginfo("Force subsriber received a wrench message!")

    # Get information about the message
    # forces = wrench_msg.force 
    # moments = wrench_msg.torque
    rospy.loginfo('Got {0}'.format(wrench_msg.force.x))

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('force_subscriber', argv=sys.argv)

    # Subscribe to force data
    wrench_sub = rospy.Subscriber('wrench_measurement', Wrench, wrench_callback)
    # wrench_sub = rospy.Subscriber('wrench_measurement', Int64, wrench_callback)

    rospy.loginfo("Force subscriber node initialized.")

    rospy.spin()