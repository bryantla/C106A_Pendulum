# subscribe to encoder topic and get readings
# use control law to determine required linear velocity of pendulum
# publish command to topic

#!/usr/bin/env python

import numpy as np

import rospy
from std_msgs.msg import Float64

from collections import deque
from intera_interface import Limb
import intera_interface
from moveit_msgs.msg import RobotTrajectory

# TODO: possibly need a ros package to read the current end effector position
# and velocity, combine that with the pendulum angle and velocity, to get
# the current state of the system and compute the error

def listener():
    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/Float64 from the topic /encoder.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    rospy.Subscriber("encoder", Float64, control_law)

    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    rospy.spin()


def control_law(angle):
    """
    Computes the commanded linear velocity of end effector given the angle of
    the pendulum.

    Parameters
    ----------
    angle (std_msgs.Float64): angle of pendulum

    Computes
    -------
    velocity (std_msgs.Float64): linear velocity along one axis of end effector
    """

    angle = angle.data

    # # YOUR CODE HERE (Task 2)
    # rospy.init_node('listener', anonymous=True)
    # for i in range(len(angles)):
    #     # joint_state.position: left joints correspond to indices 2 to 8
    #     angles[i] = joint_state.position[2+i]
    #
    # # END YOUR CODE HERE
    # print(baxter_forward_kinematics_from_angles(angles))

    talker(velocity)


# publishes linear velocity to topic
def talker(velocity):
    # Create an instance of the rospy.Publisher object which we can use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/Float64 to the topic /controller_vel
    pub = rospy.Publisher('controller_vel', Float64, queue_size=10)

    # Create a timer object that will sleep long enough to result in a 10 Hz
    # publishing rate
    r = rospy.Rate(10) # 10 Hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)
        pub_vel = Float64()
        pub_vel.data = velocity

        # Publish our message to the 'encoder_angle' topic
        pub.publish(pub_vel)
        print(pub_vel)

        # Use our rate object to sleep until it is time to publish again
        r.sleep()


if __name__ == '__main__':
    # Run this program as a new node in the ROS computation graph called
    # /controller.
    rospy.init_node('controller', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # listener method.
    try:
        listener()
    except rospy.ROSInterruptException: pass
