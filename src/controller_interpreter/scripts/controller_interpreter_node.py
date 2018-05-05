#!/usr/bin/env python

# License here!

import rospy

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Joy
from controller_interpreter.msg import ArmCommand


# Universal controls
CONTROL_SCHEME_CHANGE_BUTTON = 8

# Rover drive controls
ACCELERATE_AXIS = 5
DECELERATE_AXIS = 2
STEERING_AXIS = 0
CAMERA_YAW_AXIS = 3
CAMERA_PITCH_AXIS = 4
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
SLOW_LINEAR_AXIS = 7      # U/D D-pad - for fine control when using arm
SLOW_ANGULAR_AXIS = 6     # L/R D-pad - ditto


# Program constants
COMMAND_UPDATE_RATE = 20


# Global variable to hold most up to date input data from "Joy"
# Starts in the default controller state
most_recent_joy_data = Joy()
most_recent_joy_data.axes = (0, 0, 1, 0, 0, 1, 0, 0)
most_recent_joy_data.buttons = (0,) * 11

# Global variables to hold the current control state
control_states = ["drive", "arm"]
current_control_state_index = 0
switch_button_prev_state = 0  # Might this need debouncing?


def interpreter():
    target_vel_pub = rospy.Publisher("rover_target_vel", Twist, queue_size=5)
    arm_command_pub = rospy.Publisher("rover_arm_commands", ArmCommand, queue_size=5)
    servo_yaw_pub = rospy.Publisher("servo_yaw_rate", Float32, queue_size=5)
    servo_pitch_pub = rospy.Publisher("servo_pitch_rate", Float32, queue_size=5)

    rospy.init_node("command_interpreter")
    rospy.loginfo("Node and publishers initialised")
    rate = rospy.Rate(COMMAND_UPDATE_RATE)

    while not rospy.is_shutdown():
        target_vel, arm_command, servo_yaw_command, servo_pitch_command = get_commands()
        target_vel_pub.publish(target_vel)
        arm_command_pub.publish(arm_command)
        servo_yaw_pub.publish(servo_yaw_command)
        servo_pitch_pub.publish(servo_pitch_command)
        rate.sleep()


def get_commands():
    global control_states
    global current_control_state_index
    input_axes = most_recent_joy_data.axes
    input_buttons = most_recent_joy_data.buttons

    rover_target_vel = Twist()
    arm_command = ArmCommand()
    servo_yaw_command = Float32()
    servo_pitch_command = Float32()

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
            linear_command, angular_command, servo_command = drive_interpreter(input_axes, input_buttons)

            rospy.logdebug("drive commands: linear: {}, angular: {}".format(linear_command, angular_command))

            lin_vel = Vector3(linear_command, 0, 0)
            ang_vel = Vector3(0, 0, angular_command)

            rover_target_vel.linear = lin_vel
            rover_target_vel.angular = ang_vel

            servo_yaw_command = servo_command[0]
            servo_pitch_command = servo_command[1]

        elif control_states[current_control_state_index] == "arm":
            arm_commands = arm_interpreter(input_axes, input_buttons)
            arm_command.base_rotation_velocity = arm_commands["base_rotation_velocity"]
            arm_command.arm_actuator_1_velocity = arm_commands["arm_actuator_1_velocity"]
            arm_command.arm_actuator_2_velocity = arm_commands["arm_actuator_2_velocity"]
            arm_command.wrist_rotation_velocity = arm_commands["wrist_rotation_velocity"]
            arm_command.wrist_actuator_velocity = arm_commands["wrist_actuator_velocity"]
            arm_command.gripper_velocity = arm_commands["gripper_velocity"]

            lin_vel = arm_commands["linear_velocity"]
            ang_vel = arm_commands["angular_velocity"]
            rover_target_vel.linear = Vector3(lin_vel, 0, 0)
            rover_target_vel.angular = Vector3(0, 0, ang_vel)

    except IndexError:
        rospy.logerr("Unknown command state detected, reverting to index 0")
        current_control_state_index = 0

    return rover_target_vel, arm_command, servo_yaw_command, servo_pitch_command


def drive_interpreter(axes, buttons):
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

    # Camera servo inputs
    yaw_input = axes[CAMERA_YAW_AXIS]
    pitch_input = axes[CAMERA_PITCH_AXIS]

    # Input for both
    boost = buttons[DRIVE_BOOST_BUTTON]

    # linear command should be in [-0.5, 0.5] when boost disabled and
    # in [-1, 1] with it enabled
    # negative since triggers start from 1 and go to -1
    linear_command = -0.25 * (forward_input - reverse_input) * (1 + boost)

    # angular command should have the same range
    angular_command = 0.5 * steer_input * (1 + boost)

    # Servo commands don't use boost
    servo_command = (yaw_input, pitch_input)

    return linear_command, angular_command, servo_command


def arm_interpreter(axes, buttons):
    """
    :param axes: a tuple containing the states of each of the controller axes
    :param buttons: a tuple containing the states of each of the controller buttons
    :return: a dict mapping arm velocity component names to their desired values
    """
    commands = dict()

    # First get direct commands
    slow_linear_input = axes[SLOW_LINEAR_AXIS]
    slow_angular_input = axes[SLOW_ANGULAR_AXIS]
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
    commands["linear_velocity"] = 0.2 * slow_linear_input * 0.5 * (1 + boost_input)
    commands["angular_velocity"] = 0.2 * slow_angular_input * 0.5 * (1 + boost_input)
    commands["base_rotation_velocity"]  = base_rotation_input  * 0.5 * (1 + boost_input)
    commands["arm_actuator_1_velocity"] = arm_actuator_1_input * 0.5 * (1 + boost_input)
    commands["arm_actuator_2_velocity"] = arm_actuator_2_input * 0.5 * (1 + boost_input)
    commands["wrist_rotation_velocity"] = wrist_rotation_input * 0.5 * (1 + boost_input)
    commands["wrist_actuator_velocity"] = wrist_actuator_input * 0.5 * (1 + boost_input)
    commands["gripper_velocity"] = gripper_input * 0.5 * (1 + boost_input)

    return commands


def initialise_subscriber():
    rospy.Subscriber("joy", Joy, sub_callback)
    rospy.loginfo("Subscribed to Joy node")


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
