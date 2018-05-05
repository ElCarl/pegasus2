#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from arduino_communicator.msg import EncoderCounts

m = 0

def test(i, j):
       	"""
       	Converts a tuple of encoder data into an EncoderCounts
       	ROS message and publishes it
       	"""
	global m
	encoder_publisher = rospy.Publisher("encoder_counts", EncoderCounts, queue_size=10)
	rospy.init_node("test")
	rate = rospy.Rate(10)
        encoder_counts = EncoderCounts()
        header = Header()
	
	while not rospy.is_shutdown():
       		# Timestamp is the first element
		now = rospy.get_rostime()
       		header.stamp.secs = now.secs
       		header.stamp.nsecs = now.nsecs
       		# Left wheel encoders
       		lf = i*m
       		lm = i*m
       		lr = i*m
       		encoder_counts.left_wheel_counts = [0, 0, 0]
	
	       	# Right wheel encoders
	       	rf = j*m
	       	rm = j*m	
       		rr = j*m
       		encoder_counts.right_wheel_counts = [0, 0, 0]

       		# Then the remaining encoders
       		encoder_counts.base_rotation_counts = 0
       		encoder_counts.wrist_rotation_counts = 0
       		encoder_counts.gripper_counts = 0

       		# Finally, publish the data
       		encoder_publisher.publish(encoder_counts)
		m += 1
		rate.sleep()

if __name__ == '__main__':
     	try:	    
		test(-7, 9)
	except rospy.ROSInterruptException:
	    pass
