#!/usr/bin/env python

import rospy
import serial
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

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

last_control_data = Twist()
last_control_data.linear = Vector3(0, 0, 0)
last_control_data.angular = Vector3(0, 0, 0)

def rover_target_vel_callback(data):
    global last_control_data
    last_control_data = data

def init_subscriber():
    rospy.init_node("arduino_communicator_node")
    rospy.Subscriber("rover_target_vel", Twist, rover_target_vel_callback)

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
    while ser.read() != SERIAL_READY_BYTE:  # Should this really fail like this? I think it should actually just keep raising an error but keep trying. Otherwise could be very faffy
        if (time.clock() - start_time) * 1000 > HANDSHAKE_TIMEOUT_MS:
            rospy.logerr("Handshake timed out")
            raise RuntimeError("Handshake timed out")
        ser.write(SERIAL_READY_BYTE)  # Indented line to potentially make it more robust - will keep sending SERIAL_READY_BYTE. Try unindenting if it breaks, though.
        time.sleep(0.1)
    rospy.loginfo("Handshake successful")

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
    if checksum == calc_checksum(encoder_data[:-1]):
        # publish encoder values & timestamp
    else:
        rospy.logerr("Encoder checksum failure, ignoring message")

def main():
    init_subscriber()
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

if __name__ == "__main__":
    main()
