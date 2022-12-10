#!/usr/bin/env python

# subscribe to encoder and Sawyer arm topics and get readings
# use control law to determine required linear velocity
# publish command to topic

import numpy as np
import time

import rospy
from std_msgs.msg import String, Float32
from intera_core_msgs.msg import EndpointState

class Controller(object):

    def __init__(self):
        # topic for commanded velocity
        self._talker_uOut = rospy.Publisher("controller", Float32, queue_size=10)

        # topics for encoder and end effector states
        self._listener_xP = rospy.Subscriber("/robot/limb/right/endpoint_state", \
            EndpointState, callback=self.endeff)
        self._listener_angle = rospy.Subscriber("encoder_angle", Float32, callback=self.angle)

        # state variables
        self._xInit = -0.2075
        self._x = 0
        self._xdot = 0
        self._thetaPrev = 0
        self._theta = 0
        self._thetadot = 0

        # controller variables
        self._K = [0.5477*0.65,1.3938*0.65,40.7981*0.65,9.8650*0.65] # moment inertia rod end
        # self._K = [0.5477*0.7,1.5206*0.7,30.5069*0.7,8.8050*0.7] # mass at end, L
        self._cmd_vel = 0
        self._vel_limit = 2
        self._dt = 0.01 # 100 Hz (1/100)
        self._ctrl_timer = rospy.Timer(rospy.Duration(1/100.), self.control_law)
        # topic to reset the control input to zero
        self._rezero = rospy.Subscriber("rezero_pendulum", String, callback=self.check_rezero)

    # updates position and velocity of end effector
    def endeff(self,state):
        self._x = state.pose.position.y # wrt base frame
        self._xdot = state.twist.linear.x # wrt tool frame

    # updates angle and angular velocity
    def angle(self,pub_angle):
        self._thetaPrev = self._theta
        self._theta = pub_angle.data

        # angular velocity
        self._thetadot = (self._theta-self._thetaPrev)/self._dt

    # check to see if the control input needs to be reset to zero
    def check_rezero(self,rezero):
        if (rezero.data == 'rezero'):
            self._cmd_vel = 0

    # computes commanded linear velocity of end effector given x, xdot, theta, thetadot
    def control_law(self,_):
        # control input (acceleration)
        u = self._K[0]*(self._x-self._xInit) + self._K[1]*self._xdot + \
            self._K[2]*self._theta + self._K[3]*self._thetadot

        # command velocity, check for saturation
        self._cmd_vel = self._cmd_vel + u*self._dt
        if (np.abs(self._cmd_vel) > self._vel_limit):
            self._cmd_vel = np.sign(self._cmd_vel)*self._vel_limit

        print(['theta: ', self._theta, 'cmd_vel: ', self._cmd_vel])
        self._talker_uOut.publish(self._cmd_vel)

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
