#!/usr/bin/env python

# subscribe to encoder and Sawyer arm topics and get readings
# use control law to determine required linear velocity
# publish command to topic

# TODO: find controller gains that work for our specific system parameters
# open the Mathematica notebooks to verify what was done previously

import numpy as np
import time

import rospy
from std_msgs.msg import String, Float32, Float32MultiArray
from intera_core_msgs.msg import EndpointState

class Controller(object):

    def __init__(self):
        # topic for commanded velocity, end effector position, end effector velocity
        self._talker_yOut = rospy.Publisher("controller", Float32MultiArray, queue_size=10)

        # topics for encoder and end effector states
        self._listener_xP = rospy.Subscriber("/robot/limb/right/endpoint_state", \
            EndpointState, callback=self.cart)
        self._listener_angle = rospy.Subscriber("encoder_angle", Float32, callback=self.angle)

        # state variables
        self._xInit = -0.2075
        self._x = 0
        self._xdot = 0
        self._thetaPrev = 0
        self._theta = 0
        self._thetadot = 0

        # controller variables
        self._K = [0.5477,1.5090,30.1922,8.3422]
        self._cmd_vel = 0
        self._vel_limit = 1
        self._dt = 0.01 # 100 Hz (1/100)
        self._ctrl_timer = rospy.Timer(rospy.Duration(1/100.), self.control_law)
        self._reset = rospy.Subscriber("reset_pendulum", String, callback=self.check_reset)

    # updates position and velocity of end effector
    def cart(self,state):
        self._x = state.pose.position.y # wrt base frame
        self._xdot = state.twist.linear.x # wrt tool frame

    # updates angle and angular velocity
    def angle(self,pub_angle):
        self._thetaPrev = self._theta
        self._theta = pub_angle.data + 0.01308997 # adjustment factor to compensate for lag, smooths out motion

        # angular velocity
        self._thetadot = (self._theta-self._thetaPrev)/self._dt

    def check_reset(self,reset):
        if (reset.data == 'reset'):
            self._cmd_vel = 0

    # computes commanded linear velocity of end effector given x, xdot, theta, thetadot
    def control_law(self,_):
        # control input (acceleration)
        u = self._K[0]*(self._x-self._xInit) + self._K[1]*self._xdot + \
            self._K[2]*self._theta + self._K[3]*self._thetadot
        u = 5.*self._theta

        # command velocity, check for saturation
        self._cmd_vel = self._cmd_vel + u*self._dt
        if (np.abs(self._cmd_vel) > self._vel_limit):
            self._cmd_vel = np.sign(self._cmd_vel)*self._vel_limit

        send_data = Float32MultiArray()
        send_data.data = [self._cmd_vel, self._x, self._xdot]
        print(['theta: ', self._theta, 'cmd_vel: ', send_data.data[0]])
        self._talker_yOut.publish(send_data)

if __name__ == '__main__':
    # Run this program as a new node in the ROS computation graph called
    # /controller.
    rospy.init_node('controller', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # main loop.
    try:
        ctrllr = Controller()
    except rospy.ROSInterruptException:
        pass

    # runs in infinite loop until shutdown signal is received
    rospy.spin()
