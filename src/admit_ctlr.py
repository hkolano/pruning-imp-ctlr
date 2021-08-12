#!/usr/bin/env python

# Hannah Kolano kolanoh@oregonstate.edu
#
# Proxy for pruning project: subscribes to force data from UR5 
#
# Last modified 8/11/2021 by Hannah

import rospy
import sys
import numpy as np

from geometry_msgs.msg import Wrench, Vector3Stamped, Vector3, WrenchStamped

class AdmitCtlr():

    def __init__(self):
        '''
        Set up subscriber to the force torque sensor
        '''

        # Subscribe to wrist torque topic
        self.wrench_sub = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)
        # Publish velocities to robot
        self.vel_pub = rospy.Publisher('/velproxy', Vector3Stamped, queue_size=5)
        rospy.loginfo("Force subscriber node initialized.")

        # Set up parameters
        # Desired (reference) wrench
        self.des_wrench = np.array([0, 0, 0, 0, 0, -1])
        # Controller gain 
        self.Kf = .1
        # Selection matrix
        self.l = np.diag([1, 0, 0, 0, 1, 1])
	# Velocity limit
	self.vel_lim = 0.01 # 1 cm/s

        self.vel = Vector3Stamped()
        self.vel.header.stamp = rospy.Time.now()
        self.vel.header.frame_id = 'tool0_controller'
        self.vel.vector = Vector3(0.0, 0.0, 0.0)
	rospy.loginfo("Finished initializing admit ctlr node.")

    def impose_vel_limit(self, vel):
        if vel > self.vel_lim:
            output_vel = self.vel_lim
        elif vel < -self.vel_lim:
            output_vel = -self.vel_lim
        else:
            output_vel = vel
        return output_vel 

    def show_ctlr_direction(self, vely, velz):
        if vely > 0:
	        dir1 = "up"
        else:
	        dir1 = "down"
        if velz > 0:
	        dir2 = "forward"
        else:
	        dir2 = "backward"
        rospy.loginfo("Moving {} and {}".format(dir1, dir2))

    def wrench_callback(self, wrench_msg):
        """
        Callback function to deal with incoming wrench messages
        """
        # rospy.loginfo("Force subscriber received a wrench message!")

        # Write the wrench_msg into an array
        w = wrench_msg.wrench
        wrench_vec = np.array([w.torque.x, w.torque.y, w.torque.z, w.force.x, w.force.y, w.force.z])
        rospy.logdebug('New wrench. \n Y force: {0} \n Z force: {1} \n X moment: {2}'.format(wrench_vec[4], wrench_vec[5], wrench_vec[0]))
        # idealized velocities
        vel_des = -self.Kf*np.dot(self.l,self.des_wrench-wrench_vec)
        vel_y = vel_des[4]
        vel_z = vel_des[5]
        vel_y_limited = self.impose_vel_limit(vel_y)
        vel_z_limited = self.impose_vel_limit(vel_z)
	
        # Set up the velocity command
        self.vel.header.stamp = rospy.Time.now()
        # rospy.loginfo(d_xdes[5])
        self.vel.vector = Vector3(0.0, vel_y_limited, vel_z_limited)
        self.vel_pub.publish(self.vel)
        rospy.logdebug("Published vels: \n {}".format(self.vel.vector))
        self.show_ctlr_direction(vel_y_limited, vel_z_limited)

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('imp_ctlr', argv=sys.argv)

    ctlr = AdmitCtlr()

    rospy.spin()
