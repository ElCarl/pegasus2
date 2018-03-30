#!/usr/bin/env python

import rospy
import serial
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from arduino_communicator.msg import EncoderCounts

# Debug constants
DEBUG_MODE = False

# Serial constants
ARDUINO_PORT_BASE = "/dev/ttyACM"
BAUDRATE = 38400

# Program constants
HANDSHAKE_WARNING_MS = 5000
HANDSHAKE_TIMEOUT_MS = 20000
TIMEOUT_MS = 5000             # If no message is received for this long, restart comms TODO: actually implement!
COMMAND_UPDATE_RATE = 20
ENCODER_DATA_BYTE   = b'\xfa'  # FA=250
SERIAL_READY_BYTE   = b'\xfb'  # FB=251
BEGIN_MESSAGE_BYTE  = b'\xfc'  # FC=252
ARM_MESSAGE_BYTE    = b'\xfd'  # FD=253
DRIVE_MESSAGE_BYTE  = b'\xfe'  # FE=254
END_MESSAGE_BYTE    = b'\xff'  # FF=255

# Encoder constants
N_ENCS = 9
ENCODER_MESSAGE_TIMEOUT_MS = 50
L_FRONT_MOTOR_ENCODER = 1
L_MID_MOTOR_ENCODER   = 2
L_REAR_MOTOR_ENCODER  = 3
R_FRONT_MOTOR_ENCODER = 4
R_MID_MOTOR_ENCODER   = 5
R_REAR_MOTOR_ENCODER  = 6
BASE_ROTATE_MOTOR_ENCODER = 7
WRIST_ROTATE_MOTOR_ENCODER = 8
GRIPPER_MOTOR_ENCODER = 9

handshake_time = 0

last_control_data = Twist()
last_control_data.linear = Vector3(0, 0, 0)
last_control_data.angular = Vector3(0, 0, 0)

last_encoder_data = ()


class RoverCommand:
    drive_command = None
    arm_command   = None

    def __init__(self):
        lin_cmd = Vector3(0, 0, 0)
        ang_cmd = Vector3(0, 0, 0)
        self.drive_command = Twist(lin_cmd, ang_cmd)
        # TODO add other commands - need to create ROS message

    def update_drive_command(self, vel_topic_data):
        """
        This method should be passed as the ROS subscriber callback
        for the rover_target_vel topic
        """
        self.drive_command = vel_topic_data

    def update_arm_command(self, arm_topic_data):
        """
        This method should be passed as the ROS subscriber callback
        for the arm movement commands topic
        """
        raise NotImplementedError
        self.arm_command = arm_topic_data


class RoverController:
    last_command = None
    motor_controller = None
    encoder_publisher = None
    def __init__(self, motor_controller):
        self.motor_controller = motor_controller
        motor_controller.encoder_callback = self.publish_enc_counts
        self.command = RoverCommand()
        rospy.loginfo("RoverController instance initialised")

    def publish_enc_counts(self, encoder_data):
        """
        Converts a tuple of encoder data into an EncoderCounts
        ROS message and publishes it
        """
        if not encoder_publisher:
            rospy.logerr_throttle(10, "encoder_publisher not"
                                      "initialised, cannot publish")
            return  # Abort publishing

        encoder_counts = EncoderCounts()
        header = Header()

        # Timestamp is the first element
        arduino_timestamp_ms = encoder_data[0]  # Time since the handshake
        timestamp_ms = (self.motor_controller.handshake_time * 1000) + arduino_timestamp_ms
        timestamp_s  = int(timestamp_ms / 1000)
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
        encoder_counts.left_wheel_counts = [rf, rm, rr]
        
        # Then the remaining encoders
        encoder_counts.base_rotation_counts  = encoder_data[BASE_ROTATE_MOTOR_ENCODER]
        encoder_counts.wrist_rotation_counts = encoder_data[WRIST_ROTATE_MOTOR_ENCODER]
        encoder_counts.gripper_counts        = encoder_data[GRIPPER_MOTOR_ENCODER]
        
        # Finally, publish the data
        self.encoder_publisher.publish(encoder_counts)

    def init_publisher(self):
        self.encoder_publisher = rospy.Publisher("encoder_counts", EncoderCounts,
                                                 queue_size=50)
        rospy.loginfo("encoder_counts publisher initialised")


class MotorController:
    serial_conn = None
    handshake_time = None
    encoder_callback = None
    def __init__(self, base_port_name=ARDUINO_PORT_BASE, baudrate=BAUDRATE):
        self.base_port_name = base_port_name
        self.baudrate = baudrate
        self.port_num = 0
        self.init_serial()
        self.serial_handshake()

    def init_serial(self, retry_limit=9):
        while True:
            self.full_port_name = self.base_port_name + str(self.port_num)
            try:
                self.serial_conn = serial.Serial(self.full_port_name, self.baudrate)
            except OSError:
                rospy.logwarn("Arduino not found at %s, trying next", full_port_name)
                port_num += 1
                if port_num > retry_limit:
                    rospy.logfatal("Arduino not found on any ACM port up to %d!",
                                   retry_limit)
                    raise RuntimeError("Arduino connection unsuccessful")
            else:
                break

    def serial_handshake(self):
        if not self.serial_conn:
            rospy.logfatal("Not connected to motor controller board!")
            raise RuntimeError("Motor controller board serial connection does not exist")
        start_time = time.clock()
        while self.serial_conn.read() != SERIAL_READY_BYTE:
            if (time.clock() - start_time) * 1000 > HANDSHAKE_TIMEOUT_MS:
                rospy.logerr("Handshake timed out")
                raise RuntimeError("Handshake timed out")
            self.serial_conn.write(SERIAL_READY_BYTE)
            time.sleep(0.1)
        self.handshake_time = time.time()  # Ensure that time.time returns an
                                           # acceptably accurate number - as
                                           # long as it's ~0.1s accurate it
                                           # should be ok
        rospy.loginfo("Handshake successful")

    def send_commands(self, command_struct):
        """
        Writes the desired commands in command_struct to the device
        over serial. The command_struct *must* be the exact same as
        the struct to receive it on the other device!
        Currently, it is simply 8 bytes:
        lin_vel, ang_vel, base_rotate, actuator_1_move,
        actuator_2_move, wrist_rotate, actuator_wrist_move, gripper_move
    
        A command to control the servo will most likely be added at
        some point, probably another byte
        """
        self.serial_conn.write(BEGIN_MESSAGE_BYTE)
        command_data = struct.pack("<8B", *command_struct)
        # Send struct length - must convert len(command_data) to bytearray
        self.serial_conn.write(bytearray((len(command_data),)))
        self.serial_conn.write(command_data)
        self.serial_conn.write(END_MESSAGE_BYTE)

    def read_serial_data(self):
        if self.serial_conn.in_waiting == 0:
            return
        # else
        rec_byte = self.serial_conn.read()
        if rec_byte == ENCODER_DATA_BYTE:
            self.read_encoder_data()
        else:
            rospy.logerr("Unknown serial message type byte %b"
                         "received from motor controller", rec_byte)

    def read_encoder_data(self):
        data_str = ""
        while True:
            rec_byte = self.serial_conn.read()
            if rec_byte != END_MESSAGE_BYTE:
                data_str += rec_byte
            else:
                break
        # Use the Python struct library to interpret the data
        # '<' enforces little-endianness
        # L9lB means:
        #   L  - one unsigned long (timestamp)
        #   9l - nine signed longs (encoder counts)
        #   B  - one unsigned byte, interpreted as an integer (checksum)
        encoder_data = struct.unpack("<L9lB", data_str)
        checksum = encoder_data[-1]
        if checksum == calc_checksum(data_str[:-1]):
            self.encoder_callback(encoder_data)
        else:
            rospy.logerr("Encoder checksum failure, ignoring message")



###################################
###### Module-level functions #####
###################################

def calc_checksum(char_string):
    """
    Calculate a simple checksum of a string of chars/bytes by taking
    the XOR of all of them
    """
    checksum = 0
    str_len = len(char_string)
    format_str = "<{}B".format(str_len)
    chars = struct.unpack(format_str, char_string)
    for char in chars:
        checksum ^= char
    return checksum



#############################
##### Main Program Flow #####
#############################

def main_old():
    init_subscriber()
    init_publisher()
    ser = init_serial()
    rate = rospy.Rate(COMMAND_UPDATE_RATE)
    try:
        while True:
            lin_vel = last_control_data.linear.x
            ang_vel = last_control_data.angular.z
            write_velocity_commands(ser, lin_vel, ang_vel)
            if ser.in_waiting > 0:
                receive_serial_data(ser)

            rate.sleep()
    finally:
        write_velocity_commands(ser, 0, 0)  # Send a 0 to all motors
        ser.close()

def main_oop():
    motor_controller = MotorController()
    rover_controller = RoverController()
    rover_controller.init_publisher()
    # then main loop code

if __name__ == "__main__":
    main_old()



####################################################################
##### Putting all old code here while refactoring into classes #####
####################################################################

def write_command_struct(ser, command_struct):
    """
    Writes the desired commands in command_struct to the device over ser.
    The command_struct *must* be the exact same as the struct to receive it on the other device!
    Currently, it is simply 8 bytes:
    lin_vel
    ang_vel
    base_rotate
    actuator_1_move
    actuator_2_move
    wrist_rotate
    actuator_wrist_move
    gripper_move

    A command to control the servo will most likely be added at some point, probably another byte
    """
    ser.write(BEGIN_MESSAGE_BYTE)
    command_data = struct.pack("<8B", *command_struct)
    # Send struct length - must convert len(command_data) to bytearray
    ser.write(bytearray((len(command_data),)))
    ser.write(command_data)
    ser.write(END_MESSAGE_BYTE)

def write_velocity_commands(ser, linear_velocity, angular_velocity):
    # velocities should be in [-1,1]
    # Convert normalised velocity commands to be in range [0,200]
    lin_vel = int(100 * (1 + linear_velocity))
    ang_vel = int(100 * (1 + angular_velocity))
    # Convert commands to bytes
    lin_msg = bytearray((lin_vel,))
    ang_msg = bytearray((ang_vel,))
    ser.write(BEGIN_MESSAGE_BYTE)
    ser.write(DRIVE_MESSAGE_BYTE)
    ser.write(lin_msg)
    ser.write(ang_msg)
    ser.write(END_MESSAGE_BYTE)
    if DEBUG_MODE:
        time.sleep(0.01)
        rec_data = ser.read(ser.inWaiting())
        rospy.logdebug(rec_data)

def init_serial():
    port_num = 0
    while True:
        # Since the arduino serial port sometimes changes, we should check
        # several of them before concluding that the Arduino is unreachable
        full_port_name = ARDUINO_PORT_BASE + str(port_num)
        try:
            ser = serial.Serial(full_port_name, BAUDRATE)
        except OSError:
            rospy.logwarn("Arduino not found at %s, trying next", full_port_name)
            port_num += 1
            if port_num > 9:
                rospy.logerr("Arduino not found on any ACM port up to 9!")
                raise RuntimeError("Arduino connection unsuccessful")
        else:
            break
    # Run handshake - init will then return serial object once it is available
    arduino_handshake(ser) 
    ser.flush()
    return ser

def arduino_handshake(ser):
    # Ensures that arduino board is present and ready to receive commands
    start_time = time.clock()
    while ser.read() != SERIAL_READY_BYTE:  # Should this really fail like
                                            # this? I think it should actually
                                            # just keep raising an error but
                                            # keep trying. Otherwise could be very faffy
        if (time.clock() - start_time) * 1000 > HANDSHAKE_TIMEOUT_MS:
            rospy.logerr("Handshake timed out")
            raise RuntimeError("Handshake timed out")
        ser.write(SERIAL_READY_BYTE)  # Indented line to potentially make it more
                                      # robust - will keep sending SERIAL_READY_BYTE.
                                      # Try unindenting if it breaks, though.
        time.sleep(0.1)
    global handshake_time
    handshake_time = time.time()  # Ensure that time.time returns an acceptably
                                  # accurate number - as long as it's ~0.1s
                                  # accurate it should be ok
    rospy.loginfo("Handshake successful")

def rover_target_vel_callback(data):
    global last_control_data
    last_control_data = data

def receive_serial_data(ser):
    rec_byte = ser.read()
    if rec_byte == ENCODER_DATA_BYTE:
        receive_encoder_data(ser)
    else:
        rospy.logwarn("Unknown serial byte %b received from Arduino", rec_byte)

def receive_encoder_data(ser):
    data_str = ""
    start_time_ms = time.clock() * 1000
    while True:
        rec_byte = ser.read()
        if rec_byte != END_MESSAGE_BYTE:
            data_str += rec_byte
        else:
            break
    # Use the Python struct library to interpret the data
    # '<' enforces little-endianness
    # L9lB means:
    #   L  - one unsigned long (timestamp)
    #   9l - nine signed longs (encoder counts)
    #   B  - one unsigned byte, interpreted as an integer (checksum)
    encoder_data = struct.unpack("<L9lB", data_str)
    checksum = encoder_data[-1]
    if checksum == calc_checksum(data_str[:-1]):
        encoder_pub.publish(encoder_message_from_data(encoder_data))
    else:
        rospy.logerr("Encoder checksum failure, ignoring message")

def init_subscriber():
    rospy.init_node("arduino_communicator_node")
    rospy.Subscriber("rover_target_vel", Twist, rover_target_vel_callback)
    rospy.loginfo("subscribed to rover_target_vel")

def init_publisher():
    global encoder_pub
    encoder_pub = rospy.Publisher("encoder_counts", EncoderCounts, queue_size=50)
    rospy.loginfo("encoder_counts publisher initialised")


def encoder_message_from_data(encoder_data):
    """
    Converts a tuple of encoder data into an EncoderCounts ROS message
    """
    encoder_counts = EncoderCounts()
    
    header = Header()

    # Timestamp is the first element
    arduino_timestamp_ms = encoder_data[0]  # Time since the handshake
    timestamp_ms = (handshake_time * 1000) + arduino_timestamp_ms
    timestamp_s  = int(timestamp_ms / 1000)
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
    encoder_counts.left_wheel_counts = [rf, rm, rr]
    
    # Then the remaining encoders
    encoder_counts.base_rotation_counts  = encoder_data[BASE_ROTATE_MOTOR_ENCODER]
    encoder_counts.wrist_rotation_counts = encoder_data[WRIST_ROTATE_MOTOR_ENCODER]
    encoder_counts.gripper_counts        = encoder_data[GRIPPER_MOTOR_ENCODER]
    
    return encoder_counts
