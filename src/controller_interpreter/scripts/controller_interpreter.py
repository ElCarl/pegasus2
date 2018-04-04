#!/usr/bin/env python

# License here!

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from control_msgs.msg import GripperCommand


# Universal controls
TEMP_CONTROL_SCHEME_TOGGLE_BUTTON = 8

# Rover drive controls - rough and ready!
TEMP_ACCEL_AXIS = 5
TEMP_DECEL_AXIS = 2
TEMP_STEER_AXIS = 0
TEMP_BOOST_BUTTON_DRV = 4

# Rover arm controls - again, temporary until proper control config set up
TEMP_BASE_ROTATE_AXIS = 0
TEMP_ACT1_AXIS = 1
TEMP_ACT2_AXIS = 4
TEMP_WRIST_TWIST_AXIS = 3
TEMP_WRIST_RAISE_BUTTON = 3
TEMP_WRIST_LOWER_BUTTON = 0
TEMP_GRIPPER_CLOSE_BUTTON = 2
TEMP_GRIPPER_OPEN_BUTTON = 1
TEMP_BOOST_BUTTON_ARM = 4


# Program constants
COMMAND_UPDATE_RATE = 10


# Global variable to hold most up to date input data from "Joy"
# Starts in the default controller state
most_recent_joy_data = Joy()
most_recent_joy_data.axes = (0, 0, 1, 0, 0, 1, 0, 0)
most_recent_joy_data.buttons = (0,) * 11

# Global variables to hold the current control state
control_states = ["drive", "arm"]
current_control_state_index = 0


class ControlMapping:
    def __init__(self, mapping):
        """
        control mapping is a dict: cmap[command_name] = button_index [or axis_index]
        TODO: Currently makes lots of assumptions about what is an axis or a button. Should spend
        some time later cleaning this up to make changing mappings much less prone to error.
        Thoughts: Two possible control types: single button or +/-. Single buttons only accept
        buttons as commands; +/- can either accept an axis or an ordered pair of buttons. 
        """
        cmap = dict()
        cmap["rover_drive_forward"] = None
        cmap["rover_drive_backward"] = None
        cmap["rover_drive_steer"] = None
        cmap["rover_drive_boost"] = None
        cmap["arm_base_rotate"] = None
        cmap["arm_actuator1_move"] = None
        cmap["arm_actuator2_move"] = None
        cmap["arm_wrist_twist"] = None
        cmap["arm_wrist_raise"] = None
        cmap["arm_wrist_lower"] = None
        cmap["arm_gripper_close"] = None
        cmap["arm_gripper_open"] = None

        for key in mapping.keys():
            cmap[key] = mapping[key]

        for key in cmap.keys():
            if type(cmap[key]) != Control:
                rospy.logerr("Control %s is of type %s but must be of type %s",
                             key, str(type(key)), str(type(Control)))
            if cmap[key] is None:
                rospy.logwarn("Control %s is unbound", key)

        self.mapping = cmap


class Control:
    def __init__(self, is_axis, control_indices):
        """
        control_indices is a tuple of 1 or 2 control indices
        """
        self.is_axis = is_axis
        if self.is_axis:
            self.control_axis_index = control_indices
        else:
            self.control_button_negative = control_indices[0]
            self.control_button_positive = control_indices[1]


def interpreter():
    target_vel_pub = rospy.Publisher("rover_target_vel", Twist, queue_size=5)
    gripper_pub = rospy.Publisher("gripper_command", GripperCommand, queue_size=5)
    rospy.init_node("command_interpreter")
    rate = rospy.Rate(COMMAND_UPDATE_RATE)

    while not rospy.is_shutdown():
        target_vel, gripper = get_commands()
        target_vel_pub.publish(target_vel)
        gripper_pub.publish(gripper)
        rate.sleep()


def get_commands():
    input_axes = most_recent_joy_data.axes
    input_buttons = most_recent_joy_data.buttons

    # Get drive commands from controller inputs (TEMP)
    linear_command, angular_command = temp_drive_interpreter(input_axes, input_buttons)

    # print("linear: {}, angular: {}".format(linear_command, angular_command))

    twist = Twist()

    lin_vel = Vector3(linear_command, 0, 0)
    ang_vel = Vector3(0, 0, angular_command)

    twist.linear = lin_vel
    twist.angular = ang_vel

    gripper = GripperCommand()

    gripper.position = 0.0

    # Negative values are ignored - this should be negative unless a gripper force sensor is implemented
    gripper.max_effort = -1

    return twist, gripper


def temp_drive_interpreter(axes, buttons):
    """
    Given the current state of the axes and buttons, returns the
    desired rover velocity (normalised).
    linear: ((forward - backward) / 2) * 0.5 * (1 + boost)
    """
    # Linear inputs
    forward_input = axes[TEMP_ACCEL_AXIS]
    reverse_input = axes[TEMP_DECEL_AXIS]

    # Steering inputs
    steer_input = axes[TEMP_STEER_AXIS]

    # Input for both
    boost = buttons[TEMP_BOOST_BUTTON_DRV]

    # linear command should be in [-0.5, 0.5] when boost disabled and
    # in [-1, 1] with it enabled
    # negative since triggers start from 1 and go to -1
    linear_command = -0.25 * (forward_input - reverse_input) * (1 + boost)

    # angular command should have the same range
    angular_command = 0.5 * steer_input * (1 + boost)

    return linear_command, angular_command


def temp_arm_interpreter(axes, buttons):
    raise NotImplementedError


def initialise_subscriber():
    rospy.Subscriber("joy", Joy, sub_callback)


def sub_callback(data):
    global most_recent_joy_data
    most_recent_joy_data = data
    # print "Received data: " + str(data.axes) + "/" + str(data.buttons)


if __name__ == "__main__":
    try:
        initialise_subscriber()
        interpreter()
    except rospy.ROSInterruptException:
        pass  
