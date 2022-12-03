#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

import rospy
from sensor_msgs.msg import JointState


def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,8)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[ 0.0076, 0.0001, -1.0000],
                  [-0.7040, 0.7102, -0.0053],
                  [ 0.7102, 0.7040,  0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)
    g_st_0 = np.eye(4)
    g_st_0[0:3,0:3] = R
    g_st_0[0:3,3] = qs[0:3,7]

    xis = np.ndarray((6,7))

    for i in range(7):
        xi = np.zeros(6)
        omega = ws[0:3,i]
        q = qs[0:3,i]
        xi[3:] = omega
        xi[0:3] = -kfs.skew_3d(omega) @ q

        xis[:,i] = xi

    g_st = kfs.prod_exp(xis, joint_angles) @ g_st_0
    
    return g_st


# Define the method which contains the node's main functionality
def listener():

    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/String from the topic /chatter_talk.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    rospy.Subscriber("robot/joint_states", JointState, \
        baxter_forward_kinematics_from_joint_state)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)
    rospy.init_node('listener', anonymous=True)
    for i in range(len(angles)):
        # joint_state.position: left joints correspond to indices 2 to 8
        angles[i] = joint_state.position[2+i]

    # END YOUR CODE HERE
    print(baxter_forward_kinematics_from_angles(angles))


if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called
    # /listener_<id>, where <id> is a randomly generated numeric string. This
    # randomly generated name means we can start multiple copies of this node
    # without having multiple nodes with the same name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    listener()