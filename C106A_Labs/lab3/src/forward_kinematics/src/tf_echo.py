import tf2_ros
import sys
import rospy
import tf2_geometry_msgs


# Python's syntax for a main() method
if __name__ == '__main__':

	rospy.init_node('echo')

	# source frame (left_hand) transformed back into target frame (base)
	target_frame, source_frame = sys.argv[-2], sys.argv[-1]

	tfBuffer = tf2_ros.Buffer()
	tfListener = tf2_ros.TransformListener(tfBuffer)

	# need a while loop because communication is asynchronous
	while not rospy.is_shutdown():
		try:
			trans = tfBuffer.lookup_transform(target_frame, source_frame, rospy.Time())
			print(trans.transform)
		except (tf2_ros.LookupException, tf2_ros.ConnectivityException, \
			tf2_ros.ExtrapolationException):
			print('Error')