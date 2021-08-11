#!/usr/bin/env python

# Proxy for pruning project:
# Publishes wrench data as a stand-in for the UR-5
#
# Last modified 8/11/2021 by Hannah

import rospy
import sys
from numpy import sin, cos, pi 

from geometry_msgs.msg import Wrench, Vector3
from std_msgs.msg import Int64

class WrenchGenerator():
    '''
    Publishes wrenches to a topic as a proxy for the actual measurements
    '''
    def __init__(self):
        # Set up the pubslisher
        self.wrench_pub = rospy.Publisher('wrench_measurement', Wrench, queue_size=10)
        # self.wrench_pub = rospy.Publisher('wrench_measurement', Int64, queue_size=10)
        rospy.loginfo("Force Generator node started")

        rate = rospy.Rate(1)

        wrench = Wrench()
        wrench.force.x = 0.0
        wrench.force.y = 0.0
        wrench.force.z = 0.0
        wrench.torque.x = 0.0
        wrench.torque.y = 0.0
        wrench.torque.z = 0.0

        # Send messages until the node is shut down
        while not rospy.is_shutdown():
            # Publish the value of the counter
            self.wrench_pub.publish(wrench)

            rospy.loginfo('Published {0}'.format(wrench))

            rate.sleep()

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('force_generator', argv=sys.argv)

    generator = WrenchGenerator()

    rospy.spin()

