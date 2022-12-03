# temporary launch file
# DOES NOT WORK

import os

os.system("rosrun intera_interface enable_robot.py -e")
os.system("python encoder.py")
os.system("python controller.py")
os.system("python actuation.py")