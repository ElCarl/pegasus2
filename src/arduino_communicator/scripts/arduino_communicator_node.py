#!/usr/bin/env python

import rospy
import serial
import time

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

# Debug constants
DEBUG_MODE = True

# Serial constants
ARDUINO_PORT = "/dev/ttyACM4"
BAUDRATE = 38400

# Program constants
HANDSHAKE_WARNING_MS = 5000
HANDSHAKE_TIMEOUT_MS = 20000
COMMAND_UPDATE_RATE = 20
SERIAL_READY_BYTE  = b'\xfb'  # FB=251
BEGIN_MESSAGE_BYTE = b'\xfc'  # FC=252
ARM_MESSAGE_BYTE = b'\xfd'    # FD=253
DRIVE_MESSAGE_BYTE = b'\xfe'  # FE=254
END_MESSAGE_BYTE = b'\xff'    # FF=255

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
    ser = serial.Serial(ARDUINO_PORT, BAUDRATE)
    # Run handshake - init will then return serial object once it is available
    arduino_handshake(ser) 
    ser.flush()
    return ser

def arduino_handshake(ser):
    # Ensures that arduino board is present and ready to receive commands
    # Might not actually implement this if it's not required...
    while ser.read() != SERIAL_READY_BYTE:
        time.sleep(0.01)
    ser.write(SERIAL_READY_BYTE)

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
        print rec_data,

def main():
    ser = init_serial()
    init_subscriber()
    rate = rospy.Rate(COMMAND_UPDATE_RATE)
    try:
        while True:
            lin_vel = last_control_data.linear.x
            ang_vel = last_control_data.angular.z
            write_velocity_commands(ser, lin_vel, ang_vel)
            rate.sleep()
    finally:
        write_velocity_commands(ser, 0, 0)  # Send a 0 to all motors
        ser.close()

if __name__ == "__main__":
    main()
