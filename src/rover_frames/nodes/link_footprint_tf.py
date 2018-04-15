#!/usr/bin/env python
import rospy

# to get commandline arguments
import sys

#Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
  
    rospy.init_node('link_footprint_tf_node')
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    static_transformStamped = geometry_msgs.msg.TransformStamped()
  
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "base_footprint"
    static_transformStamped.child_frame_id = "base_link"
  
    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.35

    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0
 
    broadcaster.sendTransform(static_transformStamped)
    rospy.spin()
