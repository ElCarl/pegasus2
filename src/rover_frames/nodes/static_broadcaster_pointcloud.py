#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

#Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('pointcloud_static_broadcaster')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
  
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "kinect2_ir_optical_frame"
    static_transformStamped.child_frame_id = "pointcloud_frame"
  
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0

    quat = tf.transformations.quaternion_from_euler(-1.571, 0.0, -1.571, 'rzyx')
    static_transformStamped.transform.rotation.x = quat[0]
    static_transformStamped.transform.rotation.y = quat[1]
    static_transformStamped.transform.rotation.z = quat[2]
    static_transformStamped.transform.rotation.w = quat[3]
 
    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
