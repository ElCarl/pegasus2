#!/usr/bin/env python  
import rospy
import PyKDL

import math
import tf2_ros
import geometry_msgs.msg
import sensor_msgs.msg

def map_base_gps_tf(data):
    location = [data.latitude, data.longitude, data.altitude]
    start_point = [38.373, -110.714, 1309]
    dloc[0] = location[0] - start_point[0]
    dloc[1] = location[1] - start_point[1]
    dloc[2] = location[2] - start_point[2]

def map_base_imu_tf(data):
    orientation = data.orientation
    start = [0.0, 0.0, 0.0, 1.0]
    dor[0] = orientation[0] - start[0]
    dor[1] = orientation[1] - start[1]
    dor[2] = orientation[2] - start[2]
    dor[3] = orientation[3] - start[3]

if __name__ == '__main__':
    rospy.init_node('map_odom_tf')

    br = tf2_ros.TransformBroadcaster()
    tr = geometry_msgs.msg.TransformStamped()

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    dloc = [0.0, 0.0, 0.0]
    rospy.Subscriber('/fix', sensor_msgs.msg.NavSatFix, map_base_gps_tf)
    dor = [0.0, 0.0, 0.0, 1.0]
    rospy.Subscriber('/imu', sensor_msgs.msg.Imu, map_base_imu_tf)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        try:
    	    t = tfBuffer.lookup_transform("odom", "base_link", rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
	tr.header.stamp = rospy.Time.now()
	tr.header.frame_id = "map"
	tr.child_frame_id = "odom"

	tr.transform.translation.x = dloc[0] - t.transform.translation.x
	tr.transform.translation.y = dloc[1] - t.transform.translation.y
	tr.transform.translation.z = dloc[2] - t.transform.translation.z
	q1 = PyKDL.Rotation.Quaternion(dor[0], dor[1], dor[2], dor[3])
	q2 = PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w)

	c = q1 * q2.Inverse()
	qd = c.GetQuaternion()
	tr.transform.rotation.x = qd[0]
	tr.transform.rotation.y = qd[1]
	tr.transform.rotation.z = qd[2]
	tr.transform.rotation.w = qd[3]

	br.sendTransform(tr)
    	rate.sleep()
