#!/usr/bin/env python

# License here!

import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from controller_interpreter.msg import ArmCommand


# Universal controls
CONTROL_SCHEME_CHANGE_BUTTON = 8

# Rover drive controls - rough and ready!
ACCELERATE_AXIS = 5
DECELERATE_AXIS = 2
STEERING_AXIS = 0
DRIVE_BOOST_BUTTON = 4

# Rover arm controls
BASE_ROTATE_AXIS = 0      # LS L/R
ARM_ACTUATOR_1_AXIS = 1   # LS U/D
ARM_ACTUATOR_2_AXIS = 4   # RS U/D
WRIST_ROTATE_AXIS = 3     # RS L/R
WRIST_RAISE_BUTTON = 3    # Y
WRIST_LOWER_BUTTON = 0    # A
GRIPPER_CLOSE_BUTTON = 2  # X
GRIPPER_OPEN_BUTTON = 1   # B
ARM_BOOST_BUTTON = 4      # LB


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
switch_button_prev_state = 0  # Might this need debouncing?


#class ControlMapping:
#    def __init__(self, mapping):
#        """
#        control mapping is a dict: cmap[command_name] = button_index [or axis_index]
#        TODO: Currently makes lots of assumptions about what is an axis or a button. Should spend
#        some time later cleaning this up to make changing mappings much less prone to error.
#        Thoughts: Two possible control types: single button or +/-. Single buttons only accept
#        buttons as commands; +/- can either accept an axis or an ordered pair of buttons.
#        """
#        cmap = dict()
#        cmap["rover_drive_forward"] = None
#        cmap["rover_drive_backward"] = None
#        cmap["rover_drive_steer"] = None
#        cmap["rover_drive_boost"] = None
#        cmap["arm_base_rotate"] = None
#        cmap["arm_actuator1_move"] = None
#        cmap["arm_actuator2_move"] = None
#        cmap["arm_wrist_twist"] = None
#        cmap["arm_wrist_raise"] = None
#        cmap["arm_wrist_lower"] = None
#        cmap["arm_gripper_close"] = None
#        cmap["arm_gripper_open"] = None
#
#        for key in mapping.keys():
#            cmap[key] = mapping[key]
#
#        for key in cmap.keys():
#            if type(cmap[key]) != Control:
#                rospy.logerr("Control %s is of type %s but must be of type %s",
#                             key, str(type(key)), str(type(Control)))
#            if cmap[key] is None:
#                rospy.logwarn("Control %s is unbound", key)
#
#        self.mapping = cmap


#class Control:
#    def __init__(self, is_axis, control_indices):
#        """
#        control_indices is a tuple of 1 or 2 control indices
#        """
#        self.is_axis = is_axis
#        if self.is_axis:
#            self.control_axis_index = control_indices
#        else:
#            self.control_button_negative = control_indices[0]
#            self.control_button_positive = control_indices[1]


def interpreter():
    target_vel_pub = rospy.Publisher("rover_target_vel", Twist, queue_size=5)
    arm_command_pub = rospy.Publisher("rover_arm_commands", ArmCommand, queue_size=5)
    rospy.init_node("command_interpreter")
    rate = rospy.Rate(COMMAND_UPDATE_RATE)

    while not rospy.is_shutdown():
        target_vel, arm_command = get_commands()
        target_vel_pub.publish(target_vel)
        arm_command_pub.publish(arm_command)
        rate.sleep()


def get_commands():
    global control_states
    global current_control_state_index
    input_axes = most_recent_joy_data.axes
    input_buttons = most_recent_joy_data.buttons

    rover_target_vel = Twist()
    arm_command = ArmCommand()

    global switch_button_prev_state
    # If the change control button has been pressed
    if input_buttons[CONTROL_SCHEME_CHANGE_BUTTON] == 1 and switch_button_prev_state == 0:
        # Then change the control scheme
        current_control_state_index += 1
        current_control_state_index %= len(control_states)  # Ensure that result is within index
        rospy.loginfo("Control scheme switched to %s",control_states[current_control_state_index])
    switch_button_prev_state = input_buttons[CONTROL_SCHEME_CHANGE_BUTTON]

    try:
        if control_states[current_control_state_index] == "drive":
            # Get drive commands from controller inputs (TEMP)
            linear_command, angular_command = temp_drive_interpreter(input_axes, input_buttons)

            rospy.logdebug("drive commands: linear: {}, angular: {}".format(linear_command, angular_command))

            lin_vel = Vector3(linear_command, 0, 0)
            ang_vel = Vector3(0, 0, angular_command)

            rover_target_vel.linear = lin_vel
            rover_target_vel.angular = ang_vel

        elif control_states[current_control_state_index] == "arm":
            arm_commands = temp_arm_interpreter(input_axes, input_buttons)
            arm_command.base_rotation_velocity = arm_commands["base_rotation_velocity"]
            arm_command.arm_actuator_1_velocity = arm_commands["arm_actuator_1_velocity"]
            arm_command.arm_actuator_2_velocity = arm_commands["arm_actuator_2_velocity"]
            arm_command.wrist_rotation_velocity = arm_commands["wrist_rotation_velocity"]
            arm_command.wrist_actuator_velocity = arm_commands["wrist_actuator_velocity"]
            arm_command.gripper_velocity = arm_commands["gripper_velocity"]

    except IndexError:
        rospy.logerr("Unknown command state detected, reverting to index 0")
        current_control_state_index = 0

    return rover_target_vel, arm_command


def temp_drive_interpreter(axes, buttons):
    """
    Given the current state of the axes and buttons, returns the
    desired rover velocity (normalised).
    linear: ((forward - backward) / 2) * 0.5 * (1 + boost)
    """
    # Linear inputs
    forward_input = axes[ACCELERATE_AXIS]
    reverse_input = axes[DECELERATE_AXIS]

    # Steering inputs
    steer_input = axes[STEERING_AXIS]

    # Input for both
    boost = buttons[DRIVE_BOOST_BUTTON]

    # linear command should be in [-0.5, 0.5] when boost disabled and
    # in [-1, 1] with it enabled
    # negative since triggers start from 1 and go to -1
    linear_command = -0.25 * (forward_input - reverse_input) * (1 + boost)

    # angular command should have the same range
    angular_command = 0.5 * steer_input * (1 + boost)

    return linear_command, angular_command


def temp_arm_interpreter(axes, buttons):
    """
    :param axes: a tuple containing the states of each of the controller axes
    :param buttons: a tuple containing the states of each of the controller buttons
    :return: a dict mapping arm velocity component names to their desired values
    """
    arm_commands = dict()

    # First get direct commands
    base_rotation_input = axes[BASE_ROTATE_AXIS]
    arm_actuator_1_input = axes[ARM_ACTUATOR_1_AXIS]
    arm_actuator_2_input = axes[ARM_ACTUATOR_2_AXIS]
    wrist_rotation_input = axes[WRIST_ROTATE_AXIS]
    boost_input = buttons[ARM_BOOST_BUTTON]

    # Then interpret multi-button commands
    wrist_raise_input = buttons[WRIST_RAISE_BUTTON]
    wrist_lower_input = buttons[WRIST_LOWER_BUTTON]
    wrist_actuator_input = wrist_raise_input - wrist_lower_input
    gripper_open_input = buttons[GRIPPER_OPEN_BUTTON]
    gripper_close_input = buttons[GRIPPER_CLOSE_BUTTON]
    gripper_input = gripper_open_input - gripper_close_input

    # [command] * 0.5 * (1 + boost_input) halves command speed when boost button is not pressed
    arm_commands["base_rotation_velocity"]  = base_rotation_input  * 0.5 * (1 + boost_input)
    arm_commands["arm_actuator_1_velocity"] = arm_actuator_1_input * 0.5 * (1 + boost_input)
    arm_commands["arm_actuator_2_velocity"] = arm_actuator_2_input * 0.5 * (1 + boost_input)
    arm_commands["wrist_rotation_velocity"] = wrist_rotation_input * 0.5 * (1 + boost_input)
    arm_commands["wrist_actuator_velocity"] = wrist_actuator_input * 0.5 * (1 + boost_input)
    arm_commands["gripper_velocity"] = gripper_input * 0.5 * (1 + boost_input)

    return arm_commands


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
