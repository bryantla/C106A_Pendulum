# subscribe to command topic
# compute inverse kinematics for commanded linear velocity
# send joint velocities to Sawyer

#!/usr/bin/env python

import numpy as np
from math import pi,radians,sin,cos
import time

import rospy
import intera_interface
import modern_robotics as mr
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point

# TODO: need to create a working launch file to launch all of the node .py files

class Actuation(object):

    def __init__(self):
        s10 = sin(radians(10))
        c10 = cos(radians(10))
        # Sawyer joint control
        self._angles = [0, 0, pi/2, -pi/2, pi/2, pi/2, pi/2+s10]
        self._limb = intera_interface.Limb("right")
        self._start_joint_angles = {"right_j0":self._angles[0], "right_j1":self._angles[1], \
            "right_j2":self._angles[2], "right_j3":self._angles[3], "right_j4":self._angles[4], \
            "right_j5":self._angles[5], "right_j6":self._angles[6]}
        self._limb.move_to_joint_positions(self._start_joint_angles)

        # topics for controller output and joint state
        self._listener_yOut = rospy.Subscriber("controller", Point, self.get_y)
        self._listener_js = rospy.Subscriber("/robot/joint_states", JointState, self.get_js)

        # inverse kinematics
        # you could compute these directly from q and w, like we did in Lab 3
        self._joint_twists = \ # twists of the joint axes
            np.array([[ s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603],
                      [-c10, -s10, 0., -0.9345*s10,  0.9345*c10,      0.],
                      [  0.,   0., 1., -0.0322*s10,  0.0322*c10,      0.],
                      [-c10, -s10, 0., -0.5345*s10,  0.5345*c10,      0.],
                      [  0.,   0., 1.,  0.1363*s10, -0.1363*c10,      0.],
                      [-c10, -s10, 0., -0.1345*s10,  0.1345*c10,      0.],
                      [  0.,   0., 1.,          0.,          0.,      0.]]).T
        self._g0 = np.array([[  0.,   0., 1., 1.0155], # initial SE(3) configuration of end effector
                             [-c10, -s10, 0., 0.1603],
                             [ s10, -c10, 0.,  0.317],
                             [  0.,   0., 0.,     1.]])
        # find SE(3) configuration of end effector when moved to joint coordinates
        self._goal = mr.FKinBody(self._g0, self._joint_twists, self._start_joint_angles)
        self._goal[0:3,0:3] = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        self._js = []

        # controller variables
        self._cmd_vel = 0
        self._x = -.2075
        self._xdot = 0
        self._count = 0
        self._act_timer = rospy.Timer(rospy.Duration(1/100.), self.move)
        self._too_fast = True

    # get end effector position, commanded velocity, end effector velocity from controller
    def get_y(self,yOut):
        self._xdot = yOut.x
        self._cmd_vel = yOut.y
        self._x = yOut.z

    # get current joint positions
    def get_js(self,joint_state):
        self._js = joint_state.position[1:-1]

    # move arm according to control input
    def move_arm(self,_):
        g_d = self._goal # desired goal
        js_curr = self._js # current joint state

        # calculate transform from current to desired configuration: g(t)^-1 * gd
        trans = np.dot(mr.TransInv(mr.FKinBody(self._g0, self._joint_twists, js_curr)), g_d)
        # convert to desired twist (body velocity) by taking log
        Vb = mr.se3ToVec(mr.MatrixLog6(trans))

        # check for physical limits and update velocity
        if (self._x < .55 and self._x > -.7):
            Vb[3] = self._cmd_vel
        elif (self._x > .55 or self._x < -.7):
            self._too_fast = True

        if (self._count < pi/2 and self._too_fast == True):
            Vb[3] = self.xdot*cos(self._count) + Vb[3]*sin(self._count/4)
            self._count = self._count + .015
        elif (self._count >= pi/2 and self._count < 2*pi and self._too_fast == True):
            Vb[3] =  Vb[3]*sin(self._count/4)
            self._count = self._count + .015
        elif (self._count >= 2*pi and self._too_fast == True):
            self._count = 0
            self._too_fast = False


        # calculate body Jacobian at current configuration
        Jb = mr.JacobianBody(self._joint_twists, js_curr)
        # calculate joint angle velocity from body Jacobian
        qdot = np.dot(np.linalg.pinv(Jb), Vb)
        joint_cmd = {"right_j0":qdot[0], "right_j1":qdot[1], "right_j2":qdot[2],
                         "right_j3": qdot[3], "right_j4":qdot[4], "right_j5":qdot[5], "right_j6":qdot[6]}
        # move Sawyer arm
        self._limb.set_joint_velocities(joint_cmd)

# move Sawyer arm into initial configuration and wait for user input before proceeding
def initialize():
    # initial joint angles
    angles = [0, 0, pi/2, -pi/2, pi/2, pi/2, pi/2+s10]
    start_joint_angles = {"right_j0":angles[0], "right_j1":angles[1], \
        "right_j2":angles[2], "right_j3":angles[3], "right_j4":angles[4], \
        "right_j5":angles[5], "right_j6":angles[6]}

    # move Sawyer arm
    limb = intera_interface.Limb("right")
    limb.move_to_joint_positions(start_joint_angles)

    # wait for user input
    time.sleep(2)
    input('Move pendulum to vertical equilibrium and press <Enter>:')

if __name__ == '__main__':
    # Run this program as a new node in the ROS computation graph called
    # /actuation.
    rospy.init_node('actuation', anonymous=True)
    initialize()

    # Check if the node has received a signal to shut down. If not, run the
    # main loop.
    try:
        actr = Actuation()
    except rospy.ROSInterruptException:
        pass

    # runs in infinite loop until shutdown signal is received
    rospy.spin()
