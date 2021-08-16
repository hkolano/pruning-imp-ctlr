#!/usr/bin/env python

import rospy
import sys
import numpy as np
import matplotlib.pyplot as plt

from scipy import signal
from geometry_msgs.msg import Wrench, Vector3Stamped, Vector3, WrenchStamped

class ForceFilter():

    def __init__(self, kernel):
        self.kernel = kernel
	    # subscribe to wrench topic from UR5
        self.wrench_sub = rospy.Subscriber('/wrench', WrenchStamped, self.wrench_callback)
	    # Publish to 'filtered wrench' topic
        self.wrench_pub = rospy.Publisher('/wrench_filtered', WrenchStamped, queue_size=10)
        
        self.new_wrench = WrenchStamped()

        self.f_y_queue = np.zeros(kernel)
        self.f_z_queue = np.zeros(kernel)
        rospy.loginfo("Filter running.")

        samplerate = 500 #Hz
        nyq = 0.5*samplerate
        cut = 20/nyq 
        # N, Wn = signal.buttord(1, 5, 3, 40, fs=500.0)
        # N, Wn = signal.buttord(20, .125, 3, 40, 'True')
        # rospy.loginfo('N = {}, wn = {}'.format(N, Wn))
        # self.sos = signal.butter(N, Wn, analog='True', output='sos')
        self.sos = signal.butter(3, 20*2*3.14, 'low', output='sos', analog=True)
        # b, a = signal.butter(3, 20*2*3.14, 'low', analog=True)
        # w, h = signal.freqs(b, a, np.logspace(-1,3,500))

        # plt.semilogx(w, 20*np.log10(abs(h)))
        # plt.title('Butterworth filter')
        # plt.xlabel('Freq')
        # plt.ylabel('Amp [dB]')
        # plt.grid(which='both', axis='both')
        # plt.show()
        self.counter = 0

    def wrench_callback(self, wrench_msg):
        """
        Callback function to deal with incoming wrench messages; publishes filtered wrench 
        """
        # rospy.loginfo("Wrench filter received a wrench message!")

        # Write the wrench_msg into an array
        w = wrench_msg.wrench
        self.f_y_queue = np.append(self.f_y_queue, w.force.y)
        self.f_z_queue = np.append(self.f_z_queue, w.force.z)
        self.f_y_queue = np.delete(self.f_y_queue, 0)
        self.f_z_queue = np.delete(self.f_z_queue, 0)

        self.counter +=1 

        if self.counter > self.kernel*2:

            [filt_y, filt_z] = self.median_filter()
            # [filt_y, filt_z] = self.sos_filter()

            # rospy.loginfo("filtered values: {} and {}".format(filt_y, filt_z))
            # wrench_vec = np.array([w.torque.x, w.torque.y, w.torque.z, w.force.x, w.force.y, w.force.z])
            # rospy.logdebug('New wrench. \n Y force: {0} \n Z force: {1} \n X moment: {2}'.format(wrench_vec[4], wrench_vec[5], wrench_vec[0]))
        
            # Set up the wrench output
            self.new_wrench.header = wrench_msg.header
            # self.new_wrench.header.stamp = rospy.Time.now()
            # forward the torques (not used for the controller)
            self.new_wrench.wrench.torque = w.torque
            self.new_wrench.wrench.force.x = w.force.x
            self.new_wrench.wrench.force.y = filt_y
            self.new_wrench.wrench.force.z = filt_z
            # rospy.loginfo("publishing new wrench")
            self.wrench_pub.publish(self.new_wrench)

    def median_filter(self):
        filt_y = np.mean(self.f_y_queue)
        filt_z = np.mean(self.f_z_queue)
        return filt_y, filt_z

    def sos_filter(self):
        rospy.loginfo("original data: {}".format(self.f_y_queue))
        plt.plot(np.linspace(0,self.kernel, self.kernel), self.f_y_queue)
        filt_y = signal.sosfiltfilt(self.sos, self.f_y_queue)
        rospy.loginfo("new data: {}".format(filt_y))
        plt.plot(np.linspace(0,self.kernel, self.kernel), filt_y)
        # plt.plot(np.linspace(0,self.kernel, self.kernel), filt_y)
        # rospy.loginfo("OG: {}".format(self.f_y_queue))
        # rospy.loginfo("new: {}".format(filt_y))
        plt.show()
        # sos_z = signal.butter(2, 20, output='sos', fs=500.0)
        filt_z = signal.sosfiltfilt(self.sos, self.f_z_queue)
        return filt_y[-2], filt_z[-2]

if __name__ == '__main__':

    # Initialize node
    rospy.init_node('force_filter', argv=sys.argv)

    # input: kernel size
    filter = ForceFilter(51)

    rospy.spin()
