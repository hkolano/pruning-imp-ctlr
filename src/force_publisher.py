#!/usr/bin/env python

# Proxy for pruning project:
# Publishes wrench data as a stand-in for the UR-5
#
# Last modified 8/11/2021 by Hannah

import rospy
import sys
from random import uniform

from geometry_msgs.msg import WrenchStamped

class WrenchGenerator():
    '''
    Publishes wrenches to a topic as a proxy for the actual measurements
    '''
    def __init__(self):
        # Set up the pubslisher
        self.wrench_pub = rospy.Publisher('wrench', WrenchStamped, queue_size=10)
        # self.wrench_pub = rospy.Publisher('wrench_measurement', Int64, queue_size=10)
        rospy.loginfo("Force Generator node started")

        rate = rospy.Rate(10)

        wrench_stamped = WrenchStamped()
        w = wrench_stamped.wrench
        w.force.x = 0.0
        w.torque.y = 0.0
        w.torque.z = 0.0
        wrench_stamped.header.frame_id = 'tool0_controller'

        # Send messages until the node is shut down
        while not rospy.is_shutdown():
            # Publish the value of the counter
            w.force.y = uniform(-2, 2)
            w.force.z = uniform(-1, 1)
            w.torque.x = uniform(-.1, .1)
            wrench_stamped.header.stamp = rospy.Time.now()
            self.wrench_pub.publish(wrench_stamped)

            # rospy.loginfo('Published {0}'.format(wrench))

            rate.sleep()

if __name__ == '__main__':
    # Initialize the node
    rospy.init_node('force_generator', argv=sys.argv)

    generator = WrenchGenerator()

    rospy.spin()

