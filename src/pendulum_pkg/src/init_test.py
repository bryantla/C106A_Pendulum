import numpy as np
from math import pi,radians,sin,cos
import time, threading

import rospy
import intera_interface
from intera_interface import CHECK_VERSION
import modern_robotics as mr
from std_msgs.msg import Float32, Float32MultiArray
from sensor_msgs.msg import JointState

if __name__ == '__main__':
    rospy.init_node('test', anonymous=True)

    rs = intera_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled
    rs.enable()
    
    angles = [0, 0, pi/2, -pi/2, pi/2, pi/2, pi/2+sin(radians(10))]
    start_joint_angles = {"right_j0":angles[0], "right_j1":angles[1], \
        "right_j2":angles[2], "right_j3":angles[3], "right_j4":angles[4], \
        "right_j5":angles[5], "right_j6":angles[6]}

    # move Sawyer arm
    limb = intera_interface.Limb("right")
    limb.move_to_joint_positions(start_joint_angles)

    # wait for user input
    input('Move pendulum to vertical equilibrium and press <Enter>:')

    rospy.spin()