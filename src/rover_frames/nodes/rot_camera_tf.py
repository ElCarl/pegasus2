#!/usr/bin/env python  
import rospy
import tf2_ros
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler
import geometry_msgs.msg
from std_msgs.msg import Float32
import math

def callback(data):
	global orient
        orient = quaternion_from_euler(0, 0, data.data)

if __name__ == '__main__':
    rospy.init_node('dynamic_tf2_broadcaster')
    global orient
    orient = [0.0, 0.0, 0.0, 1.0]
    rospy.Subscriber("camera_yaw", Float32, callback)
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.frame_id = "base_link"
    t.child_frame_id = "rot_camera"

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():

        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 0.36
        t.transform.translation.y = 0.0
        t.transform.translation.z = 1.03
        t.transform.rotation.x = orient[0]
        t.transform.rotation.y = orient[1]
        t.transform.rotation.z = orient[2]
        t.transform.rotation.w = orient[3]

        br.sendTransform(t)
        rate.sleep()
