#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
from intera_interface import gripper as robot_gripper


def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')
    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    right_gripper = robot_gripper.Gripper('right_gripper')
    right_gripper.calibrate()
    rospy.sleep(2.0)
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Amir coords
        init_coords = [0.856, -0.023, 0.134, 0.989, -0.032, -0.145, 0.009]
        final_coords = [0.766, -0.425, 0.143, 0.963, -0.266, 0.035, -0.011]

        # Alan coords
        init_coords = [0.755, -0.023, 0.48, 0.996, 0.035, 0.052, -0.065]
        final_coords = [0.755, -0.193, 0.186, 0.984, -0.016, 0.155, -0.085]


        # 1) MOVE TO INITIAL POSITION
        request.ik_request.pose_stamped.pose.position.x = init_coords[0]
        request.ik_request.pose_stamped.pose.position.y = init_coords[1]
        request.ik_request.pose_stamped.pose.position.z = init_coords[2]
        request.ik_request.pose_stamped.pose.orientation.x = init_coords[3]
        request.ik_request.pose_stamped.pose.orientation.y = init_coords[4]
        request.ik_request.pose_stamped.pose.orientation.z = init_coords[5]
        request.ik_request.pose_stamped.pose.orientation.w = init_coords[6]

        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            user_input = ''
            while user_input != 'y':
                # Plan IK
                plan = group.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
                
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # 2) CLOSE GRIPPER
        # rospy.sleep(1)
        # right_gripper.calibrate()
        right_gripper.close()
        rospy.sleep(1)

        # 3) MOVE TO FINAL POSITION
        request.ik_request.pose_stamped.pose.position.x = final_coords[0]
        request.ik_request.pose_stamped.pose.position.y = final_coords[1]
        request.ik_request.pose_stamped.pose.position.z = final_coords[2]
        request.ik_request.pose_stamped.pose.orientation.x = final_coords[3]
        request.ik_request.pose_stamped.pose.orientation.y = final_coords[4]
        request.ik_request.pose_stamped.pose.orientation.z = final_coords[5]
        request.ik_request.pose_stamped.pose.orientation.w = final_coords[6]

        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # TRY THIS
            # Setting just the position without specifying the orientation
            ###group.set_position_target([0.5, 0.5, 0.0])

            user_input = ''
            while user_input != 'y':
                 # Plan IK
                plan = group.plan()
                user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                group.execute(plan[1])
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        # 4) OPEN GRIPPER
        # right_gripper.calibrate()
        right_gripper.open()
        rospy.sleep(1)

        # i = 0
        # while True:
        #     if i == 0:
        #         # Set the desired orientation for the end effector HERE
        #         request.ik_request.pose_stamped.pose.position.x = init_coords[0]
        #         request.ik_request.pose_stamped.pose.position.y = init_coords[1]
        #         request.ik_request.pose_stamped.pose.position.z = init_coords[2]
        #         request.ik_request.pose_stamped.pose.orientation.x = init_coords[3]
        #         request.ik_request.pose_stamped.pose.orientation.y = init_coords[4]
        #         request.ik_request.pose_stamped.pose.orientation.z = init_coords[5]
        #         request.ik_request.pose_stamped.pose.orientation.w = init_coords[6]
        #     else:
        #         request.ik_request.pose_stamped.pose.position.x = final_coords[0]
        #         request.ik_request.pose_stamped.pose.position.y = final_coords[1]
        #         request.ik_request.pose_stamped.pose.position.z = final_coords[2]
        #         request.ik_request.pose_stamped.pose.orientation.x = final_coords[3]
        #         request.ik_request.pose_stamped.pose.orientation.y = final_coords[4]
        #         request.ik_request.pose_stamped.pose.orientation.z = final_coords[5]
        #         request.ik_request.pose_stamped.pose.orientation.w = final_coords[6]
        
        #     try:
        #         # Send the request to the service
        #         response = compute_ik(request)
                
        #         # Print the response HERE
        #         print(response)
        #         group = MoveGroupCommander("right_arm")

        #         # Setting position and orientation target
        #         group.set_pose_target(request.ik_request.pose_stamped)

        #         # TRY THIS
        #         # Setting just the position without specifying the orientation
        #         ###group.set_position_target([0.5, 0.5, 0.0])

        #         # Plan IK and execute
        #         group.go()
        #         # for k in range(2000):
        #         #     group.go()
        #         #     rate.sleep()

        #         if i == 0:
        #             right_gripper.close()
        #         else:
        #             right_gripper.open()
        #         rospy.sleep(1.0)
                
        #     except rospy.ServiceException as e:
        #         print("Service call failed: %s"%e)
        #     i = 1 - i

# Python's syntax for a main() method
if __name__ == '__main__':
    main()