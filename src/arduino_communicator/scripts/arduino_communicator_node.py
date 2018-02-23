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
TIMEOUT_MS = 5000             # If no message is received for this long, restart comms
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
    while ser.read() != SERIAL_READY_BYTE:
        time.sleep(0.01)
        if (time.clock() - start_time) * 1000 > HANDSHAKE_TIMEOUT_MS:
            rospy.logerr("Handshake timed out")
            raise RuntimeError("Handshake timed out")
    ser.write(SERIAL_READY_BYTE)
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

def main():
    init_subscriber()
    ser = init_serial()
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
