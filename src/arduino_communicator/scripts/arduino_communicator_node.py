#!/usr/bin/env python

import rospy
import serial
import time
import struct

from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from arduino_communicator.msg import EncoderCounts
from controller_interpreter.msg import ArmCommand

# Debug constants
DEBUG_MODE = False

# Serial constants
ARDUINO_PORT_BASE = "/dev/ttyACM"
BAUDRATE = 1000000
SERIAL_RETRY_LIMIT = 30
READ_TIMEOUT_S = 0.01
WRITE_TIMEOUT_S = 0.01

# Program constants
HANDSHAKE_TIMEOUT_MS = 5000
TIMEOUT_MS = 5000             # If no message is received for this long, restart comms TODO: actually implement!
COMMAND_UPDATE_RATE = 20
DEBUG_BYTE = b'\xf8'  # F8=248
BOARD_STATUS_BYTE = b'\xf9'  # F9=249
ENCODER_DATA_BYTE = b'\xfa'  # FA=250
SERIAL_READY_BYTE = b'\xfb'  # FB=251
BEGIN_MESSAGE_BYTE = b'\xfc'  # FC=252
ARM_MESSAGE_BYTE = b'\xfd'  # FD=253
DRIVE_MESSAGE_BYTE = b'\xfe'  # FE=254
END_MESSAGE_BYTE = b'\xff'  # FF=255

# Command constants
COMMAND_STRUCT = {
    # Not strictly used, but good for reference
    "lin_vel": int,
    "ang_vel": int,
    "base_rot": int,
    "arm_act_1": int,
    "arm_act_2": int,
    "wrist_rot": int,
    "wrist_act": int,
    "gripper": int,
    "servo_yaw_pos_deg": int,
    "servo_pitch_pos_deg": int
}
COMMAND_STRUCT_LEN = len(COMMAND_STRUCT)
COMMAND_STRUCT_FORMAT = "<{}B".format(COMMAND_STRUCT_LEN)  # Only valid when all commands are bytes

# Servo constants
MAX_SERVO_SPEED_DPS = 10
MIN_YAW_SERVO_ANGLE = 0
MAX_YAW_SERVO_ANGLE = 180
MIN_PITCH_SERVO_ANGLE = 55
MAX_PITCH_SERVO_ANGLE = 180

# Encoder constants
NUM_ENCODERS = 7
ENCODER_MESSAGE_TIMEOUT_MS = 50
L_FRONT_MOTOR_ENCODER = 4
L_MID_MOTOR_ENCODER = 5
L_REAR_MOTOR_ENCODER = 6
R_FRONT_MOTOR_ENCODER = 2
R_MID_MOTOR_ENCODER = 1
R_REAR_MOTOR_ENCODER = 0
#BASE_ROTATE_MOTOR_ENCODER = 
#WRIST_ROTATE_MOTOR_ENCODER = 
GRIPPER_MOTOR_ENCODER = 4

# Error codes
ERROR_CODES = {
    0: "ENCODER_STRUCT_LEN_MISMATCH",
    1: "ENCODER_STRUCT_TOO_LONG",
    2: "ENCODER_CHECKSUM_ERROR",
    3: "ENCODER_READ_ERROR",
    4: "COMMAND_FIND_START_ERROR",
    5: "COMMAND_CHECKSUM_ERROR",
    6: "NO_COMMANDS_ERROR",
    7: "PID_DISABLED"
}

handshake_time = 0


class RoverCommand:
    drive_command = None
    arm_command = None
    servo_yaw_pos = None
    servo_pitch_pos = None
    last_yaw_servo_update_time = None
    last_pitch_servo_update_time = None

    def __init__(self):
        # Initialise drive and arm commands with zeros
        self.drive_command = Twist()
        self.arm_command = ArmCommand()
        self.servo_yaw_rate = Float32(0)
        self.servo_pitch_rate = Float32(0)
        self.servo_yaw_pos = 90
        self.servo_pitch_pos = 90
        self.last_yaw_servo_update_time = time.time()
        self.last_pitch_servo_update_time = time.time()

    def update_drive_command(self, vel_topic_data):
        """
        This method should be passed as the ROS subscriber callback
        for the rover_target_vel topic
        """
        self.drive_command = vel_topic_data
        rospy.logdebug("updated drive command")

    def update_arm_command(self, arm_topic_data):
        """
        This method should be passed as the ROS subscriber callback
        for the arm movement commands topic
        """
        self.arm_command = arm_topic_data
        rospy.logdebug("updated arm command")

    def update_yaw_command(self, yaw_topic_data):
        dt = time.time() - self.last_yaw_servo_update_time
        val = yaw_topic_data.data
        self.servo_yaw_pos += MAX_SERVO_SPEED_DPS * dt * val
        self.servo_yaw_pos = constrain(self.servo_yaw_pos, MIN_YAW_SERVO_ANGLE, MAX_YAW_SERVO_ANGLE)
        self.last_yaw_servo_update_time = time.time()

    def update_pitch_command(self, pitch_topic_data):
        dt = time.time() - self.last_pitch_servo_update_time
        val = pitch_topic_data.data
        self.servo_pitch_pos += MAX_SERVO_SPEED_DPS * dt * val
        self.servo_pitch_pos = constrain(self.servo_pitch_pos, MIN_PITCH_SERVO_ANGLE, MAX_PITCH_SERVO_ANGLE)
        self.last_pitch_servo_update_time = time.time()


class RoverController:
    last_command = None
    board_interface = None
    rover_command = None
    encoder_publisher = None
    yaw_publisher = None
    pitch_publisher = None

    servo_yaw_pos = None
    servo_pitch_pos = None

    def __init__(self, board_interface, rover_command):
        """
        :param board_interface: the BoardInterface object responsible for communicating with the board
        :param rover_command: the RoverCommand object containing the most recent rover and arm velocity commands
        """
        self.board_interface = board_interface
        self.rover_command = rover_command
        board_interface.encoder_callback = self.publish_enc_counts
        self.command = RoverCommand()
        self.servo_yaw_pos = 90
        self.servo_pitch_pos = 90

        self.init_encoder_publisher()
        self.init_servo_publishers()

        rospy.loginfo("RoverController instance initialised")

    def pass_commands(self):
        """
        Passes the most recent commands from self.rover_command
        into board_interface to be sent to the board_interface
        board
        """
        rospy.logdebug("Passing commands")
        # Rover velocity commands
        lin_vel = self.rover_command.drive_command.linear.x
        ang_vel = self.rover_command.drive_command.angular.z
        # Arm velocity commands
        base_rot = self.rover_command.arm_command.base_rotation_velocity
        arm_act_1 = self.rover_command.arm_command.arm_actuator_1_velocity
        arm_act_2 = self.rover_command.arm_command.arm_actuator_2_velocity
        wrist_rot = self.rover_command.arm_command.wrist_rotation_velocity
        wrist_act = self.rover_command.arm_command.wrist_actuator_velocity
        gripper = self.rover_command.arm_command.gripper_velocity
        # Servo commands
        servo_yaw_pos_deg = self.rover_command.servo_yaw_pos
        servo_pitch_pos_deg = self.rover_command.servo_pitch_pos

        command_struct = [0] * COMMAND_STRUCT_LEN

        # Scales commands to be [0,200]
        command_struct[0] = int(100 + (100 * lin_vel))
        command_struct[1] = int(100 + (100 * ang_vel))
        command_struct[2] = int(100 + (100 * base_rot))
        command_struct[3] = int(100 + (100 * arm_act_1))
        command_struct[4] = int(100 + (100 * arm_act_2))
        command_struct[5] = int(100 + (100 * wrist_rot))
        command_struct[6] = int(100 + (100 * wrist_act))
        command_struct[7] = int(100 + (100 * gripper))
        # Servo positions are [0, 180] already
        command_struct[8] = int(servo_yaw_pos_deg)
        command_struct[9] = int(servo_pitch_pos_deg)

        rospy.logdebug("Commands of type %s", type(command_struct[0]))

        self.board_interface.send_commands(command_struct)

    def publish_enc_counts(self, encoder_data):
        """
        Converts a tuple of encoder data into an EncoderCounts
        ROS message and publishes it
        """
        if not self.encoder_publisher:
            rospy.logerr_throttle(10, "encoder_publisher not"
                                      "initialised, cannot publish")
            return  # Abort publishing

        encoder_counts = EncoderCounts()
        header = Header()

        # Timestamp is the first element
        arduino_timestamp_ms = encoder_data[0]  # Time since the handshake
        encoder_data = encoder_data[1:]  # Remove the timestamp so encoders are zero-indexed
        timestamp_ms = (self.board_interface.handshake_time * 1000) + arduino_timestamp_ms
        timestamp_s = int(timestamp_ms / 1000)
        timestamp_ns = (timestamp_ms - (1000 * timestamp_s)) * 1000000
        header.stamp.secs = timestamp_s
        header.stamp.nsecs = timestamp_ns

        # Left wheel encoders
        lf = encoder_data[L_FRONT_MOTOR_ENCODER]
        lm = encoder_data[L_MID_MOTOR_ENCODER]
        lr = encoder_data[L_REAR_MOTOR_ENCODER]
        encoder_counts.left_wheel_counts = [lf, lm, lr]

        # Right wheel encoders
        rf = encoder_data[R_FRONT_MOTOR_ENCODER]
        rm = encoder_data[R_MID_MOTOR_ENCODER]
        rr = encoder_data[R_REAR_MOTOR_ENCODER]
        encoder_counts.right_wheel_counts = [rf, rm, rr]

        # Then the remaining encoders
        ##encoder_counts.base_rotation_counts = encoder_data[BASE_ROTATE_MOTOR_ENCODER]
        ##encoder_counts.wrist_rotation_counts = encoder_data[WRIST_ROTATE_MOTOR_ENCODER]
        encoder_counts.gripper_counts = encoder_data[GRIPPER_MOTOR_ENCODER]

        # Add the header to the message
        encoder_counts.header = header

        # Finally, publish the data
        self.encoder_publisher.publish(encoder_counts)

    def publish_camera_angles(self):
        self.yaw_publisher.publish(self.rover_command.servo_yaw_pos)
        self.pitch_publisher.publish(self.rover_command.servo_pitch_pos)

    def init_encoder_publisher(self):
        self.encoder_publisher = rospy.Publisher("encoder_counts", EncoderCounts,
                                                 queue_size=50)
        rospy.loginfo("encoder_counts publisher initialised")

    def init_servo_publishers(self):
        self.yaw_publisher = rospy.Publisher("camera_yaw", Float32, queue_size=5)
        self.pitch_publisher = rospy.Publisher("camera_pitch", Float32, queue_size=5)
        rospy.loginfo("camera yaw and pitch publishers initialised")


class BoardInterface:
    serial_conn = None
    full_port_name = None
    port_num = None
    handshake_time = None
    encoder_callback = None  # Set by RoverController on initialisation

    def __init__(self, base_port_name=ARDUINO_PORT_BASE, baudrate=BAUDRATE):
        self.base_port_name = base_port_name
        self.baudrate = baudrate
        self.port_num = 0
        self.init_serial()
        self.serial_handshake()

    def init_serial(self, retry_limit=SERIAL_RETRY_LIMIT):
        pass

    def serial_handshake(self):
        pass

    def send_commands(self, command_struct):
        pass

    def read_serial_data(self):
        pass

    def read_encoder_data(self):
        pass

    def echo_message(self):
        pass


##########################
# Module-level functions #
##########################

def calc_checksum(char_string):
    """
    Calculate a simple checksum of a string of chars/bytes by taking
    the XOR of all of them
    """
    str_len = len(char_string)
    checksum = str_len
    format_str = "<{}B".format(str_len)
    chars = struct.unpack(format_str, char_string)
    for char in chars:
        checksum ^= char
    return checksum


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))

#####################
# Main Program Flow #
#####################


def main():
    # Start the ROS node
    rospy.init_node("arduino_communicator_node")

    # Create & initialise objects
    rover_command = RoverCommand()
    board_interface = BoardInterface()
    rover_controller = RoverController(board_interface, rover_command)

    # Initialise ROS subscribers
    rospy.Subscriber("rover_target_vel", Twist, rover_command.update_drive_command)
    rospy.Subscriber("rover_arm_commands", ArmCommand, rover_command.update_arm_command)
    rospy.Subscriber("servo_yaw_rate", Float32, rover_command.update_yaw_command)
    rospy.Subscriber("servo_pitch_rate", Float32, rover_command.update_pitch_command)

    rospy.loginfo("Subscribers initialised")

    # Set ROS rate
    rate = rospy.Rate(COMMAND_UPDATE_RATE)

    last_time = time.time()

    # then main loop code
    try:
        while not rospy.is_shutdown():
            if time.time() - last_time > (1.0/COMMAND_UPDATE_RATE):
                rover_controller.pass_commands()
                last_time = time.time()
            board_interface.read_serial_data()  # Should this also be encapsulated within RoverController?
            rover_controller.publish_camera_angles()
            #rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        board_interface.serial_conn.close()
        # TODO Stop all motors: implement a method in rover_controller/board_interface
        # TODO release serial object
            

if __name__ == "__main__":
    main()
