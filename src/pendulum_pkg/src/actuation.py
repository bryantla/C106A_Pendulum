#!/usr/bin/env python

# subscribe to command topic
# compute inverse kinematics for commanded linear velocity
# send joint velocities to Sawyer
    # CANNOT COMPUTE INVERSE KINEMATICS USING THE LAB 3, SINCE THAT USED LEFT HAND PARAMETERS

import numpy as np
import time

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
import core as mr # Modern Robotics code library
from std_msgs.msg import String, Float32
from sensor_msgs.msg import JointState
from intera_core_msgs.msg import EndpointState

class Actuation(object):

    def __init__(self):
        sin10 = np.sin(np.radians(10))
        cos10 = np.cos(np.radians(10))

        # Sawyer joint control
        self._angles = [0, 0, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, np.pi/2+sin10]
        self._limb = intera_interface.Limb("right")
        self._start_joint_angles = {"right_j0":self._angles[0], "right_j1":self._angles[1], \
            "right_j2":self._angles[2], "right_j3":self._angles[3], "right_j4":self._angles[4], \
            "right_j5":self._angles[5], "right_j6":self._angles[6]}

        # topics for controller output, end effector state, and joint state
        # self._listener_uOut = rospy.Subscriber("controller", Float32, callback=self.get_u)
        # self._listener_xP = rospy.Subscriber("/robot/limb/right/endpoint_state", \
        #     EndpointState, callback=self.endeff)
        # self._listener_js = rospy.Subscriber("/robot/joint_states", JointState, callback=self.get_js)

        # inverse kinematics
        self._joint_twists = \
            np.array([[ sin10, -cos10, 0., -1.0155*cos10, -1.0155*sin10, -0.1603],
                      [-cos10, -sin10, 0., -0.9345*sin10,  0.9345*cos10,      0.],
                      [    0.,     0., 1., -0.0322*sin10,  0.0322*cos10,      0.],
                      [-cos10, -sin10, 0., -0.5345*sin10,  0.5345*cos10,      0.],
                      [    0.,     0., 1.,  0.1363*sin10, -0.1363*cos10,      0.],
                      [-cos10, -sin10, 0., -0.1345*sin10,  0.1345*cos10,      0.],
                      [    0.,     0., 1.,            0.,            0.,      0.]]).T # twists of the joint axes
        self._g0 = np.array([[    0.,     0., 1., 1.0155], # initial SE(3) configuration of end effector
                             [-cos10, -sin10, 0., 0.1603],
                             [ sin10, -cos10, 0.,  0.317],
                             [    0.,     0., 0.,     1.]])
        # find SE(3) configuration of end effector when moved to joint coordinates
        self._g_eq = mr.FKinBody(self._g0, self._joint_twists, self._angles)
        self._g_eq[0:3,0:3] = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
        self._js = []
        self._Vb_prev = np.array([0,0,0,0,0,0])

        # controller variables
        self._cmd_vel = 0
        self._x = -0.2075
        self._act_timer = rospy.Timer(rospy.Duration(1/100.), self.move_arm)

        # restructuring, so that callback functions aren't called before attributes are defined
        self._listener_uOut = rospy.Subscriber("controller", Float32, callback=self.get_u)
        self._listener_xP = rospy.Subscriber("/robot/limb/right/endpoint_state", \
            EndpointState, callback=self.endeff)
        self._listener_js = rospy.Subscriber("/robot/joint_states", JointState, callback=self.get_js)

    # get commanded velocity from controller
    def get_u(self,uOut):
        self._cmd_vel = uOut.data

    # updates position of end effector
    def endeff(self,state):
        self._x = state.pose.position.y # wrt base frame

    # get current joint positions
    def get_js(self,joint_state):
        self._js = joint_state.position[1:-1]

    # move arm according to control input
    def move_arm(self,_):
        g_sd = self._g_eq # desired goal
        js_curr = self._js # current joint state

        # calculate transform from current to desired configuration: g(t)^-1 * gd
        g_st = mr.FKinBody(self._g0, self._joint_twists, js_curr)
        g_ts = mr.TransInv(g_st)
        g_td = np.dot(g_ts, g_sd)
        # convert to desired twist (body velocity) by taking log
        # ignore jumps in velocity
        xi_hat = mr.MatrixLog6(g_td)
        Vb_temp = mr.se3ToVec(xi_hat)
        if (np.linalg.norm(Vb_temp - self._Vb_prev) < 1):
            self._Vb_prev = Vb_temp
            Vb = Vb_temp
        else:
            Vb = self._Vb_prev

        # check for physical limits and update velocity
        if (self._x < -.05 and self._x > -.55):
            Vb[4] = self._cmd_vel
            Vb = np.array([0,0,0,0,self._cmd_vel,0])
            print(self._x)
            # note: looks like Vb is defined as [w, v] here
            # note: the coordinate system definitions seem to change each time the robot is run
            # v = [vx, vy, vz] = [+ towards wall/back of Sawyer, + left, + up]
            # w = [wx, wy, wz] = [roll (x), pitch (y), yaw (z)]
        elif (self._x > -.05 or self._x < -.55):
            self._too_fast = True
            Vb = np.array([0,0,0,0,0,0]) # stop the arm completely

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
    rezero = rospy.Publisher("rezero_pendulum",String, queue_size=3)

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()

    # initial joint angles
    angles = [0, 0, np.pi/2, -np.pi/2, np.pi/2, np.pi/2, np.pi+np.sin(np.radians(10))]
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
    rezero.publish('rezero')
    limb.set_joint_position_speed(1.0)

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
