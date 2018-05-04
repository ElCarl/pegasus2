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
MIN_SERVO_ANGLE = 0
MAX_SERVO_ANGLE = 180

# Encoder constants
NUM_ENCODERS = 7
ENCODER_MESSAGE_TIMEOUT_MS = 50
L_FRONT_MOTOR_ENCODER = 5
L_MID_MOTOR_ENCODER = 6
L_REAR_MOTOR_ENCODER = 7
R_FRONT_MOTOR_ENCODER = 3
R_MID_MOTOR_ENCODER = 2
R_REAR_MOTOR_ENCODER = 1
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
    6: "NO_COMMANDS_ERROR"
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
        self.servo_yaw_pos = constrain(self.servo_yaw_pos, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE)
        self.last_yaw_servo_update_time = time.time()

    def update_pitch_command(self, pitch_topic_data):
        dt = time.time() - self.last_pitch_servo_update_time
        val = pitch_topic_data.data
        self.servo_pitch_pos += MAX_SERVO_SPEED_DPS * dt * val
        self.servo_pitch_pos = constrain(self.servo_pitch_pos, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE)
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
        while True:
            self.full_port_name = self.base_port_name + str(self.port_num)
            try:
                self.serial_conn = serial.Serial(self.full_port_name, self.baudrate,
                                                 timeout=READ_TIMEOUT_S, writeTimeout=WRITE_TIMEOUT_S)
            except OSError:
                rospy.logwarn("Arduino not found at %s, trying next", self.full_port_name)
                self.port_num += 1
                if self.port_num > retry_limit:
                    rospy.logfatal("Arduino not found on any ACM port up to %d!",
                                   retry_limit)
                    raise RuntimeError("Arduino connection unsuccessful")
            else:
                rospy.loginfo("Serial connection made")
                break

    def serial_handshake(self):
        rospy.loginfo("Attempting serial handshake")
        if not self.serial_conn:
            rospy.logfatal("Not connected to motor controller board!")
            raise RuntimeError("Motor controller board serial connection does not exist")
        start_time = time.time()
        connected = False
        while not connected:
            try:
                # May be needed to ensure that this will actually send a ready byte
                self.serial_conn.write(SERIAL_READY_BYTE)
                while self.serial_conn.read() != SERIAL_READY_BYTE:
                    if (time.time() - start_time) * 1000 > HANDSHAKE_TIMEOUT_MS:
                        rospy.logerr("Handshake timed out. Try running reset_arduino.py")
                        raise RuntimeError("Handshake timed out")
                    self.serial_conn.write(SERIAL_READY_BYTE)
                    time.sleep(0.1)
                connected = True
            except serial.serialutil.SerialException:
                rospy.logerr("Serial connection error")
        self.handshake_time = time.time()

        while self.serial_conn.read() != END_MESSAGE_BYTE:
            pass

        rospy.loginfo("Handshake successful")

    def send_commands(self, command_struct):
        """
        Writes the desired commands in command_struct to the device
        over serial. The command_struct *must* be the exact same as
        the struct to receive it on the other device!
        Currently, it is 10 bytes:
        lin_vel, ang_vel, base_rotate, actuator_1_move,
        actuator_2_move, wrist_rotate, wrist_actuator_move, gripper_move
        servo_yaw, servo_pitch

        A command to control the camera servo will most likely be added
        at some point, probably another byte
        """
        rospy.logdebug_throttle(2, "sending commands: {}".format(" ".join(str(c) for c in command_struct)))
        assert(len(command_struct) == COMMAND_STRUCT_LEN)
        command_data = struct.pack(COMMAND_STRUCT_FORMAT, *command_struct)
        checksum = calc_checksum(command_data)

        try:
            self.serial_conn.write(BEGIN_MESSAGE_BYTE)
            # Send struct length - must convert len(command_data) to byte array
            self.serial_conn.write(bytearray((len(command_data),)))
            self.serial_conn.write(command_data)
            # Send checksum
            self.serial_conn.write(bytearray((checksum,)))  # Regularly times out here!
        except serial.SerialTimeoutException:
            rospy.logerr("Serial write timeout")
            # Just pass for now, ignoring this attempt and trying again later
            pass

    def read_serial_data(self):
        if self.serial_conn.inWaiting() == 0:
            return
        # else
        rec_byte = self.serial_conn.read()
        if rec_byte == ENCODER_DATA_BYTE:
            self.read_encoder_data()
        elif rec_byte == BOARD_STATUS_BYTE:
            error_code = ord(self.serial_conn.read())
            rospy.logwarn_throttle(2, ERROR_CODES[error_code]);
        elif rec_byte == DEBUG_BYTE:
            msg_len = ord(self.serial_conn.read())
            msg_bytes = self.serial_conn.read(msg_len)
            format_str = "B" * msg_len
            info = [str(c) for c in struct.unpack(format_str, msg_bytes)]
            msg = "Debug msg: " + ";".join(info)
            rospy.logwarn_throttle(2, msg)
        else:
            val = ord(rec_byte)
            rospy.logerr("Unknown serial message type byte %d "
                         "received from motor controller", val)
            self.echo_message()

    def read_encoder_data(self):
        try:
            struct_len = ord(self.serial_conn.read())
            data_str = self.serial_conn.read(struct_len)
            rec_checksum = ord(self.serial_conn.read())
        except serial.SerialTimeoutException:
            rospy.logerr("Serial timeout awaiting encoder data")
            return False

        # Use the Python struct library to interpret the data
        # '<' enforces little-endianness
        # L[n]l means:
        #   L  - one unsigned long (timestamp)
        #   [n]l - [n] signed longs (encoder counts)
        try:
            format_str = "<L{}l".format(NUM_ENCODERS)
            encoder_data = struct.unpack(format_str, data_str)
        except struct.error:
            rospy.logerr("read_encoder_data error in struct unpack. Message: %s", data_str)
            return False

        if rec_checksum == calc_checksum(data_str):
            self.encoder_callback(encoder_data)
            rospy.logdebug_throttle(2, "encoder message received")
        else:
            rospy.logerr("Encoder checksum failure, ignoring message")

    def echo_message(self):
        start_time = time.time()
        while self.serial_conn.inWaiting() < 12:
            if time.time() > start_time + 0.05:
                break
        echo = self.serial_conn.read(self.serial_conn.inWaiting())
        rospy.loginfo("Echo: %s", [ord(c) for c in echo])


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

    # then main loop code
    try:
        while not rospy.is_shutdown():
            rover_controller.pass_commands()
            board_interface.read_serial_data()  # Should this also be encapsulated within RoverController?
            rover_controller.publish_camera_angles()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        board_interface.serial_conn.close()
        # TODO Stop all motors: implement a method in rover_controller/board_interface
        # TODO release serial object
            

if __name__ == "__main__":
    main()
