#!/usr/bin/env python

# A quick program to test controller input interpretation by controlling the turtle
# node in ROS

import rospy

from geometry_msgs.msg import Twist, Vector3


def callback(data):
    global input_data
    input_data = data

cmdvel_pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)


input_data = Twist(Vector3(), Vector3())

rospy.Subscriber("rover_target_vel", Twist, callback)
rospy.init_node("turtle_controller")
rate = rospy.Rate(50)

while True:
    cmdvel_pub.publish(input_data)
    rate.sleep()
