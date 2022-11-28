# subscribe to encoder topic and get readings
# use control law to determine required linear velocity of pendulum
# publish command to topic

#!/usr/bin/env python

import numpy as np
import time

import rospy
from std_msgs.msg import Float32
from intera_core_msgs.msg import EndpointState
from geometry_msgs.msg import Point

class Controller(object):

    def __init__(self):
        # topic for end effector velocity, commanded velocity, end effector position
        self._talker_yOut = rospy.Publisher("yOut",Point, queue_size=3)

        # topics for encoder and end effector states
        self._listener_xP = rospy.Subscriber("/robot/limb/right/endpoint_state",\
            EndpointState,self.cart)
        self._listener_angle = rospy.Subscriber("encoder",Float32,self.angle)

        # state variables
        self._xInit = -0.208
        self._x = 0
        self._xdot = 0
        self._thetaPrev = 0
        self._theta = 0
        self._thetadot = 0

        # control variables
        self._cmd_vel = 0
        self._vel_limit = 1
        self._dt = 0.1 # 10 Hz (1/10)
        self._ctrl_timer = rospy.Timer(rospy.Duration(1/100.), self.control_law)

    # computes commanded linear velocity of end effector given x, xdot, theta, thetadot
    def control_law(self,t):
        # control input (acceleration)
        u = 0.5477*(self._x-self._xInit) + 1.5090*self._xdot + \
            30.1922*self._theta + 8.3422*self._thetadot

        # command velocity, check for saturation
        self._cmd_vel = self._cmd_vel + u*self._dt
        if (np.abs(self._cmd_vel) > self._vel_limit):
            self._cmd_vel = np.sign(self._cmd_vel)*self._vel_limit

        self._talker_yOut.publish(self._xdot,self._cmd_vel,self._x)

    # updates angle and angular velocity
    def angle(self,pub_angle):
        self._thetaPrev = self._theta
        self._theta = pub_angle.data

        # angular velocity
        self._thetadot = (self._theta-self._thetaPrev)/self._dt

    # updates position and velocity of end effector
    def cart(self,state):
        self._x = state.pose.position.y # wrt base frame
        self._xdot = state.twist.linear.x # wrt tool frame

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

    rospy.spin()
