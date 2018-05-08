#!/usr/bin/env python
import rospy
from math import pi, sin, cos
from arduino_communicator.msg import EncoderCounts
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
from src.encoder import Encoder
from src.pose import Pose

class odometry_node:

	def __init__(self):
	        self.leftFEncoder = Encoder()
	        self.leftMEncoder = Encoder()
	        self.leftREncoder = Encoder()
	        self.rightFEncoder = Encoder()
	        self.rightMEncoder = Encoder()
	        self.rightREncoder = Encoder()
		self.pose = Pose()
		self.lastTime = 0

    	def setTime(self, newTime):
        	self.lastTime = newTime

	def main(self):
		self.odomPub = rospy.Publisher('odometry/filtered', Odometry, queue_size=10)
		self.tfPub = TransformBroadcaster()

		rospy.init_node('odometry_node')
        	self.nodeName = rospy.get_name()
        	rospy.loginfo("{0} started".format(self.nodeName))

        
		self.ticksPerMeter = int(rospy.get_param('~ticks_per_meter', 780))
        	self.wheelSeparation = float(rospy.get_param('~wheel_separation', 0.7))
        	self.rate = float(rospy.get_param('~rate', 10.0))
        	self.baseFrameID = rospy.get_param('~base_frame_id', 'base_footprint')
        	self.odomFrameID = rospy.get_param('~odom_frame_id', 'odom')
        	self.encoderMin = int(rospy.get_param('~encoder_min', -32768))
        	self.encoderMax = int(rospy.get_param('~encoder_max', 32767))

		self.setTime(rospy.get_time())
        	rate = rospy.Rate(self.rate)
		rospy.Subscriber("encoder_counts", EncoderCounts, callback=self.callback)
		rospy.spin()

	def callback(self, data):
		self.calculate_pose(data)

	def calculate_pose(self, data):
		lc = data.left_wheel_counts #front to back
		rc = data.right_wheel_counts #same here yo
		#update left encoders
		self.leftFEncoder.update(lc[0])
		self.leftMEncoder.update(lc[1])
		self.leftREncoder.update(lc[2])
		#update right encoders
		self.rightFEncoder.update(rc[0])
		self.rightMEncoder.update(rc[1])
		self.rightREncoder.update(rc[2])
		#get Travels
		leftFTravel = self.leftFEncoder.getDelta() / self.ticksPerMeter
		leftMTravel = self.leftMEncoder.getDelta() / self.ticksPerMeter
		leftRTravel = self.leftREncoder.getDelta() / self.ticksPerMeter
		rightFTravel = (self.rightFEncoder.getDelta() / self.ticksPerMeter)
		rightMTravel = (self.rightMEncoder.getDelta() / self.ticksPerMeter)
		rightRTravel = (self.rightREncoder.getDelta() / self.ticksPerMeter)
		rospy.loginfo(leftFTravel)
		rospy.loginfo(rightFTravel)
		#time stuff
		newTime = rospy.get_time()
		deltaTime = newTime - self.lastTime
		self.setTime(newTime)
		#middle travel
		aveLT = (leftFTravel + leftMTravel + leftRTravel)/3
		aveRT = (rightFTravel + rightMTravel + rightRTravel)/3
		rospy.loginfo(aveLT)
		rospy.loginfo(aveRT)
		midTravel = (aveRT + (-1*(aveLT)))/2
		midTheta = (aveRT - (-1*(aveLT)))/self.wheelSeparation
        	if -1*aveRT == aveLT:
            		deltaX = aveLT*cos(self.pose.theta)
            		deltaY = aveLT*sin(self.pose.theta)
        	else:
			radius = midTravel / midTheta
            		# Find the instantaneous center of curvature (ICC) offset.
            		iccDx = radius*sin(self.pose.theta)
            		iccDy = -radius*cos(self.pose.theta)

            		deltaX = cos(midTheta)*iccDx + sin(midTheta)*iccDy - iccDx
            		deltaY = sin(midTheta)*iccDx + cos(midTheta)*iccDy - iccDy
        	#set poses
		self.pose.x += deltaX
        	self.pose.y += deltaY
        	self.pose.theta = (self.pose.theta + midTheta) % (2*pi)
        	self.pose.xVel = midTravel / deltaTime if deltaTime > 0 else 0.
        	self.pose.yVel = 0
        	self.pose.thetaVel = midTheta / deltaTime if deltaTime > 0 else 0.

		self.lastTime = newTime

        	now = rospy.get_rostime()

        	q = Quaternion()
        	q.x = 0
        	q.y = 0
        	q.z = sin(self.pose.theta / 2)
        	q.w = cos(self.pose.theta / 2)
       		self.tfPub.sendTransform((self.pose.x, self.pose.y, 0), (q.x, q.y, q.z, q.w),
					now, self.baseFrameID, self.odomFrameID)

	        odom = Odometry()
	        odom.header.stamp = now
	        odom.header.frame_id = self.odomFrameID
	        odom.child_frame_id = self.baseFrameID
	        odom.pose.pose.position.x = self.pose.x
	        odom.pose.pose.position.y = self.pose.y
	        odom.pose.pose.position.z = 0
	        odom.pose.pose.orientation = q
		odom.pose.covariance[0] = 1e-3
		odom.pose.covariance[7] = 1e-3
		odom.pose.covariance[14] = 1e-3
		odom.pose.covariance[21] = 100000
		odom.pose.covariance[28] = 100000
		odom.pose.covariance[35] = 1e-3

	        odom.twist.twist.linear.x = self.pose.xVel
	        odom.twist.twist.linear.y = 0
	        odom.twist.twist.angular.z = self.pose.thetaVel
		odom.twist.covariance[0] = 1e-3
		odom.twist.covariance[7] = 1e-3
		odom.twist.covariance[14] = 100000
		odom.twist.covariance[21] = 100000
		odom.twist.covariance[28] = 100000
		odom.twist.covariance[35] = 1e-3
		self.odomPub.publish(odom)

	def __getitem__(self, item):
        	return getattr(self, item)

if __name__ == '__main__':
    try:
        node = odometry_node()
        node.main()
    except rospy.ROSInterruptException:
        pass
