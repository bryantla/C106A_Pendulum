# subscribe to command topic
# compute inverse kinematics for commanded linear velocity
# send joint velocities to Sawyer

#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Float64

# TODO: need a ros package to command a linear acceleration from the control
# input

def listener():
    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/Float64 from the topic /controller.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    rospy.Subscriber("controller", Float64, inverse_kinematics)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


def inverse_kinematics(velocity):
    """
    Computes the joint velocities from the desired linear velocity, and moves
    the Sawyer arm.

    Parameters
    ----------
    velocity (std_msgs.Float64): linear velocity of end effector

    """

    velocity = velocity.data

    # # YOUR CODE HERE (Task 2)
    # rospy.init_node('listener', anonymous=True)
    # for i in range(len(angles)):
    #     # joint_state.position: left joints correspond to indices 2 to 8
    #     angles[i] = joint_state.position[2+i]
    #
    # # END YOUR CODE HERE
    # print(baxter_forward_kinematics_from_angles(angles))


if __name__ == '__main__':
    # Run this program as a new node in the ROS computation graph called
    # /actuation.
    rospy.init_node('actuation', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # listener method.
    try:
        listener()
    except rospy.ROSInterruptException: pass
