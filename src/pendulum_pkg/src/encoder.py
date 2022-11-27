# read encoder counts from serial port
# publish encoder readings to topic

#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

import rospy

# import Float64 message type from std_msgs package
from std_msgs.msg import Float64

# TODO: need to create a launch file to launch all of the node .py files
# TODO: need a main.py file like in lab 6, which initializes the Sawyer arm in
# the equivalent of some tucked position, with the gripper facing up (init node)
# make it go to the initial position and require user input to continue to the
# while not rospy.is_shutdown() main loop once the user puts the pendulum in the
# vertical/equilibrium position
# TODO: import some ros packaage to read message from the serial port

# publish angle to topic
def talker():
    # Create an instance of the rospy.Publisher object which we can use to
    # publish messages to a topic. This publisher publishes messages of type
    # std_msgs/Float64 to the topic /encoder_angle
    pub = rospy.Publisher('encoder_angle', Float64, queue_size=10)

    # Create a timer object that will sleep long enough to result in a 10 Hz
    # publishing rate
    r = rospy.Rate(10) # 10 Hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        # Construct a string that we want to publish (in Python, the "%"
        # operator functions similarly to sprintf in C or MATLAB)
        pub_angle = Float64()
        pub_angle.data = 0 #TODO: somehow read from Arduino/serial/USB to get angle

        # Publish our message to the 'encoder_angle' topic
        pub.publish(pub_angle)
        print(pub_angle)

        # Use our rate object to sleep until it is time to publish again
        r.sleep()

if __name__ == '__main__':
    # Run this program as a new node in the ROS computation graph called /encoder.
    rospy.init_node('encoder', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        talker()
    except rospy.ROSInterruptException: pass
