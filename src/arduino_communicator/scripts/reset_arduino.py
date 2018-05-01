import serial
import time

ARDUINO_PORT_BASE = "/dev/ttyACM"
RESET_BAUDRATE = 1200
RETRY_LIMIT = 30

port_num = 0
is_reset = False
serial_conn = None

print "Resetting",

while not is_reset:
    full_port_name = ARDUINO_PORT_BASE + str(port_num)
    try:
        serial_conn = serial.Serial(full_port_name, RESET_BAUDRATE)
    except OSError:
        port_num += 1
        print ".",
        if port_num > RETRY_LIMIT:
            break
    else:
        is_reset = True
    finally:
        if serial_conn is not None:
            serial_conn.close()

print ""

if is_reset:
    print "Arduino successfully reset. Allowing at least 10 seconds for the board to reset fully..."
    time.sleep(10)
else:
    print "Reset failed, board not found on any port up to {}.\nHas the board just been reset?".format(full_port_name)

