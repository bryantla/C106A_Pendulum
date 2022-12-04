#!/usr/bin/env python

# subscribe to command topic
# compute inverse kinematics for commanded linear velocity
# send joint velocities to Sawyer
    # CANNOT COMPUTE INVERSE KINEMATICS USING THE LAB 3, SINCE THAT USED LEFT HAND PARAMETERS

import numpy as np
from math import pi,radians,sin,cos
import time, threading

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
import modern_robotics as mr
from std_msgs.msg import String, Float32, Float32MultiArray
from sensor_msgs.msg import JointState

class Actuation(object):

    def __init__(self):
        self._mutex = threading.Lock()

        s10 = sin(radians(10))
        c10 = cos(radians(10))
        # Sawyer joint control
        self._angles = [0, 0, pi/2, -pi/2, pi/2, pi/2, pi/2+s10]
        self._limb = intera_interface.Limb("right")
        self._start_joint_angles = {"right_j0":self._angles[0], "right_j1":self._angles[1], \
            "right_j2":self._angles[2], "right_j3":self._angles[3], "right_j4":self._angles[4], \
            "right_j5":self._angles[5], "right_j6":self._angles[6]}

        # topics for controller output and joint state
        # self._listener_yOut = rospy.Subscriber("controller", Float32MultiArray, callback=self.get_y)
        # self._listener_js = rospy.Subscriber("/robot/joint_states", JointState, callback=self.get_js)

        # inverse kinematics
        self._joint_twists = \
            np.array([[ s10, -c10, 0., -1.0155*c10, -1.0155*s10, -0.1603],
                      [-c10, -s10, 0., -0.9345*s10,  0.9345*c10,      0.],
                      [  0.,   0., 1., -0.0322*s10,  0.0322*c10,      0.],
                      [-c10, -s10, 0., -0.5345*s10,  0.5345*c10,      0.],
                      [  0.,   0., 1.,  0.1363*s10, -0.1363*c10,      0.],
                      [-c10, -s10, 0., -0.1345*s10,  0.1345*c10,      0.],
                      [  0.,   0., 1.,          0.,          0.,      0.]]) # twists of the joint axes
        self._joint_twists = self._joint_twists.T
        self._g0 = np.array([[  0.,   0., 1., 1.0155], # initial SE(3) configuration of end effector
                             [-c10, -s10, 0., 0.1603],
                             [ s10, -c10, 0.,  0.317],
                             [  0.,   0., 0.,     1.]])
        # find SE(3) configuration of end effector when moved to joint coordinates
        self._goal = mr.FKinBody(self._g0, self._joint_twists, self._angles)
        # self._goal[0:3,0:3] = np.array([[0,-1,0],[1,0,0],[0,0,1]])
        self._goal[0:3,0:3] = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        self._js = []
        self._Vb_prev = np.array([0,0,0,0,0,0])

        # controller variables
        self._cmd_vel = 0
        self._x = -0.2075
        self._xdot = 0
        self._count = 0
        self._act_timer = rospy.Timer(rospy.Duration(1/100.), self.move_arm)
        self._too_fast = True

        # restructuring, so that callback functions aren't called before attributes are defined
        self._listener_yOut = rospy.Subscriber("controller", Float32MultiArray, callback=self.get_y)
        self._listener_js = rospy.Subscriber("/robot/joint_states", JointState, callback=self.get_js)

    # get commanded velocity, end effector position, end effector velocity from controller
    def get_y(self,yOut):
        self._cmd_vel = yOut.data[0]
        self._x = yOut.data[1]
        self._xdot = yOut.data[2]

    # get current joint positions
    def get_js(self,joint_state):
        self._js = joint_state.position[1:-1]

    # move arm according to control input
    def move_arm(self,_):
        with self._mutex:
            g_d = self._goal # desired goal
            js_curr = self._js # current joint state

        # calculate transform from current to desired configuration: g(t)^-1 * gd
        trans = np.dot(mr.TransInv(mr.FKinBody(self._g0, self._joint_twists, js_curr)), g_d)
        # convert to desired twist (body velocity) by taking log
        # ignore jumps in velocity
        Vb_temp = mr.se3ToVec(mr.MatrixLog6(trans))
        if (np.linalg.norm(Vb_temp - self._Vb_prev) < 1):
            self._Vb_prev = Vb_temp
            Vb = Vb_temp
        else:
            Vb = self._Vb_prev

        # check for physical limits and update velocity
        if (self._x < .55 and self._x > -.7):
            Vb[4] = self._cmd_vel
            # note: looks like Vb is defined as [w, v] here
            # note: the coordinate system definitions seem to change each time the robot is run
            # v = [vx, vy, vz] = [+ towards wall/back of Sawyer, + left, + up]
            # w = [wx, wy, wz] = [roll (x), pitch (y), yaw (z)]
        elif (self._x > .55 or self._x < -.7):
            self._too_fast = True

        # if (self._count < pi/2 and self._too_fast == True):
        #     Vb[3] = self._xdot*cos(self._count) + Vb[3]*sin(self._count/4)
        #     self._count = self._count + .015
        # elif (self._count >= pi/2 and self._count < 2*pi and self._too_fast == True):
        #     Vb[3] =  Vb[3]*sin(self._count/4)
        #     self._count = self._count + .015
        # elif (self._count >= 2*pi and self._too_fast == True):
        #     self._count = 0
        #     self._too_fast = False


        # calculate body Jacobian at current configuration
        Jb = mr.JacobianBody(self._joint_twists, js_curr)
        # calculate joint angle velocity from body Jacobian
        qdot = np.dot(np.linalg.pinv(Jb), Vb)
        joint_cmd = {"right_j0":qdot[0], "right_j1":qdot[1], "right_j2":qdot[2],
                         "right_j3": qdot[3], "right_j4":qdot[4], "right_j5":qdot[5], "right_j6":qdot[6]}
        # move Sawyer arm
        self._limb.set_joint_velocities(joint_cmd)
        # print(self._limb.endpoint_pose())

# move Sawyer arm into initial configuration and wait for user input before proceeding
def initialize():
    # topic to reset the control input to zero
    reset = rospy.Publisher("reset_pendulum",String, queue_size=3)

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    # initial joint angles
    angles = [0, 0, pi/2, -pi/2, pi/2, pi/2, pi+sin(radians(10))]
    start_joint_angles = {"right_j0":angles[0], "right_j1":angles[1], \
        "right_j2":angles[2], "right_j3":angles[3], "right_j4":angles[4], \
        "right_j5":angles[5], "right_j6":angles[6]}

    # move Sawyer arm at a slow velocity
    limb = intera_interface.Limb("right")
    limb.set_joint_position_speed(0.3)
    limb.move_to_joint_positions(start_joint_angles)

    # wait for user input
    input('Move pendulum to vertical equilibrium and press <Enter>:')
    print("Starting pendulum...")
    # request to reset the control input to zero before moving the arm
    reset.publish('reset')

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
