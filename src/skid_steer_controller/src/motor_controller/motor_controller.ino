#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>


// CONSTANTS

// Debug constants
const bool DEBUG_MODE = 1;

// Electrical & physical constants
const float MOTOR_MAX_SPEED = 0.8;  // Normalised - so 0.7 is 70% duty cycle rather than 0.7 m/s
const float L_MOTOR_RELATIVE_SPEEDS[] = {1, 1, 1};  // Front, mid, rear
const float R_MOTOR_RELATIVE_SPEEDS[] = {1, 1, 1};  // Front, mid, rear
const uint8_t WHEELS_PER_SIDE = 3;
const uint8_t N_ENCS = 7;
const float ROVER_LENGTH_M  = 0.9;
const float ROVER_WIDTH_M   = 0.674;
const float WHEEL_CIRCUMF_M = 0.638;
const uint8_t MOTOR_GEAR_RATIO = 24; // Motor gearbox ratio - check
const uint8_t WHEEL_GEAR_RATIO = 2;  // Motor output to wheel rotations

// PWM constants
const unsigned long PWM_I2C_CLOCKSPEED = 400000;    // I2C fast mode @ 400 kHz
const uint8_t L_FRONT_MOTOR_PWM        = 11;
const uint8_t L_MID_MOTOR_PWM          = 9;
const uint8_t L_REAR_MOTOR_PWM         = 7;
const uint8_t R_FRONT_MOTOR_PWM        = 10;
const uint8_t R_MID_MOTOR_PWM          = 8;
const uint8_t R_REAR_MOTOR_PWM         = 6;
const uint8_t BASE_ROTATE_MOTOR_PWM    = 2;
const uint8_t ARM_ACTUATOR_1_PWM       = 5;
const uint8_t ARM_ACTUATOR_2_PWM       = 4;
const uint8_t GRIPPER_MOTOR_PWM        = 0;
const uint8_t WRIST_ROTATE_MOTOR_PWM   = 1;
const uint8_t WRIST_ACTUATOR_PWM       = 3;
const uint8_t L_MOTOR_PWM_CHANNELS[]   = {L_FRONT_MOTOR_PWM, L_MID_MOTOR_PWM, L_REAR_MOTOR_PWM};
const uint8_t R_MOTOR_PWM_CHANNELS[]   = {R_FRONT_MOTOR_PWM, R_MID_MOTOR_PWM, R_REAR_MOTOR_PWM};

// General PWM constants
const uint8_t PWM_CHANNELS = 16;
const unsigned int PWM_TICKS = 4096;

// Motor board PWM constants
const uint16_t MOTOR_PWM_BOARD_FREQ_HZ = 1600;  // Max freq of PWM board is ~1.6kHz - makes an annoying noise!

// Servo board PWM constants
const uint16_t SERVO_PWM_BOARD_FREQ_HZ = 60;
const uint8_t CAMERA_YAW_PWM = 0;
const uint8_t CAMERA_PITCH_PWM = 1;

// Camera servo constants
const uint8_t SERVO_MIN_YAW_DEG = 0;
const uint8_t SERVO_MAX_YAW_DEG = 0;
const uint8_t SERVO_MIN_PITCH_DEG = 0;
const uint8_t SERVO_MAX_PITCH_DEG = 0;
const uint16_t YAW_SERVO_MIN_TICK = 102;
const uint16_t YAW_SERVO_MAX_TICK = 512;
const uint16_t PITCH_SERVO_MIN_TICK = 102;
const uint16_t PITCH_SERVO_MAX_TICK = 512;

// Serial constants
const unsigned long BRAS_BAUDRATE = 1000000;  // For comms with the Braswell chip (arduino_communicator_node)
const unsigned long TIMEOUT_MS = 5000;   // TODO - actually implement this!
const byte DEBUG_BYTE         = 248;
const byte BOARD_STATUS_BYTE  = 249;
const byte ENCODER_DATA_BYTE  = 250;
const byte SERIAL_READY_BYTE  = 251;
const byte BEGIN_MESSAGE_BYTE = 252;
const byte ARM_MESSAGE_BYTE   = 253;
const byte DRIVE_MESSAGE_BYTE = 254;
const byte END_MESSAGE_BYTE   = 255;
const uint8_t SERIAL_RX_BUFF_LEN = 64;
const uint8_t MAX_COMMAND_READ_ATTEMPTS = 64;
const uint16_t COMMAND_TIMEOUT_RESET_MS = 5000;

// SPI constants
const uint32_t SPI_CLOCKSPEED_HZ = 4000000;
const uint8_t REQUEST_DELAY_US = 75;
const uint8_t SEND_DELAY_US    = 9;
const uint8_t REQUEST_ENCODERS = 248;
const uint8_t SEND_MESSAGE     = 252;
const uint8_t END_MESSAGE      = 255;
const uint8_t SPI_MAX_READ_ATTEMPTS = 3;
const uint8_t SPI_RX_BUFF_LEN = 64;

// Encoder constants
const uint8_t ENCODER_READ_RATE_HZ = 20;
const uint8_t ENCODER_SEND_RATE_HZ = 5;  // Encodometry probably needs less data than PID
const uint8_t TICKS_PER_MOTOR_REV = 10;  // Check
const uint16_t TICKS_PER_REV = TICKS_PER_MOTOR_REV * MOTOR_GEAR_RATIO * WHEEL_GEAR_RATIO;

// Other IO constants
const uint8_t MOTOR_ENABLE_PIN = 4;  // Pin chosen at random, change as appropriate

// Error codes
const uint8_t ENCODER_STRUCT_LEN_MISMATCH = 0;
const uint8_t ENCODER_STRUCT_TOO_LONG = 1;
const uint8_t ENCODER_CHECKSUM_ERROR = 2;
const uint8_t ENCODER_READ_ERROR = 3;
const uint8_t COMMAND_FIND_START_ERROR = 4;
const uint8_t COMMAND_CHECKSUM_ERROR = 5;
const uint8_t NO_COMMANDS_ERROR = 6;

// STRUCTS

// Define the encoder data structure
struct ENCODER_DATA_STRUCTURE {
    uint32_t tick_stamp_ms;  // Timestamp of encoder counts in ms since encoder counter arduino started
    int32_t encoder_counts[N_ENCS];
};

// Define the rover commands data structure
struct ROVER_COMMAND_DATA_STRUCTURE {
    uint8_t rover_linear_velocity;
    uint8_t rover_angular_velocity;
    uint8_t base_rotation_velocity;
    uint8_t arm_actuator_1_velocity;
    uint8_t arm_actuator_2_velocity;
    uint8_t wrist_rotation_velocity;
    uint8_t wrist_actuator_velocity;
    uint8_t gripper_velocity;
    uint8_t yaw_servo_position_deg;
    uint8_t pitch_servo_position_deg;
};



// GLOBAL VARIABLES

// Handshake timings
uint32_t enc_count_handshake_time_ms;
uint32_t handshake_offset_ms;

// PWM board objects
Adafruit_PWMServoDriver motor_pwm = Adafruit_PWMServoDriver();  // Defaults to 0x40
Adafruit_PWMServoDriver servo_pwm = Adafruit_PWMServoDriver(0x41);

// Structs
ENCODER_DATA_STRUCTURE encoder_counts_struct;
ROVER_COMMAND_DATA_STRUCTURE rover_command_struct;

// Struct info
uint8_t * encoder_struct_addr = (uint8_t *)&encoder_counts_struct;
uint8_t * command_struct_addr = (uint8_t *)&rover_command_struct;
uint8_t encoder_struct_len = sizeof(encoder_counts_struct);
uint8_t command_struct_len = sizeof(rover_command_struct);

// Timings
uint32_t last_command_time_ms = 0;
uint32_t last_encoder_msg_recv_ms = 0;
uint32_t last_encoder_msg_sent_ms = 0;


// MAIN PROGRAM CODE


// Initialisation code

void setup() {
    // Pin modes
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    
    // While setting up, ensure motors cannot be used
    disable_motors();

    // Start PWMs with all motors stationary and servos at 90 degrees
    init_pwm();

    // Begin serial connection to Braswell
    Serial.begin(BRAS_BAUDRATE);

    // Wait until Braswell serial is connected before connecting to encoder counter
    while(!Serial);

    // Send the serial ready byte to indicate readiness for data while awaiting
    // readiness confirmation from Braswell chip
    Serial.write(SERIAL_READY_BYTE);
    while (Serial.read() != SERIAL_READY_BYTE) {
        Serial.write(SERIAL_READY_BYTE);
        delay(100);
    }
    Serial.write(END_MESSAGE_BYTE);
    
    enc_count_handshake_time_ms = millis();

    init_spi();

    // Once all setup is complete, allow motors to be used
    enable_motors();
}

void disable_motors() {
    // This should disable the pin connected to the PWM input of each motor driver.
    // Since we are using locked antiphase PWM drive for the motors, this will prevent
    // any current reaching the motors.
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void enable_motors() {
    // Opposite of disable_motors!
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

void init_pwm() {
    // Begin PWM
    motor_pwm.begin();
    servo_pwm.begin();

    motor_pwm.setPWMFreq(MOTOR_PWM_BOARD_FREQ_HZ);
    servo_pwm.setPWMFreq(SERVO_PWM_BOARD_FREQ_HZ);

    // Set the I2C bus speed
    Wire.setClock(PWM_I2C_CLOCKSPEED);

    // Set PWM for motors to 50% which stops motors
    for (uint8_t pwm_num = 0; pwm_num < PWM_CHANNELS; pwm_num++) {
        motor_pwm.setPWM(pwm_num, 0, PWM_TICKS / 2);
    }

    // Set PWM for servos to 90 degrees
    uint16_t yaw_tick = map(90, 0, 180, YAW_SERVO_MIN_TICK, YAW_SERVO_MAX_TICK);
    uint16_t pitch_tick = map(90, 0, 180, PITCH_SERVO_MIN_TICK, PITCH_SERVO_MAX_TICK);
    servo_pwm.setPWM(CAMERA_YAW_PWM, 0, yaw_tick);
    servo_pwm.setPWM(CAMERA_PITCH_PWM, 0, pitch_tick);
}

void init_spi() {
    // Initialise SPI settings
    SPISettings spi_settings(SPI_CLOCKSPEED_HZ, MSBFIRST, SPI_MODE0);

    // Set pin modes - may not be necessary as they may be set in SPI.begin()
    pinMode(MISO,  INPUT);  // Master in, slave out
    pinMode(MOSI, OUTPUT);  // Master out, slave in
    pinMode(SCK,  OUTPUT);  // Clock
    pinMode(SS,   OUTPUT);  // Slave select

    // Start with the SPI slave disabled
    digitalWrite(SS, HIGH);

    SPI.begin();

    // If more SPI devices were added, this could be moved elsewhere
    // to only be called when needed
    SPI.beginTransaction(spi_settings);
}


// Loop code

void loop() {
    // If any movement commands have been sent,
    if (Serial.available() >= command_struct_len + 3) {
        // and if reading them is successful,
        if (read_commands()) {
            // then set the motor velocities accordingly.
            set_motor_velocities();
            set_servo_positions();
            last_command_time_ms = millis();
        }
        else {
            // If we've not got a valid command recently
            if (millis() > last_command_time_ms + COMMAND_TIMEOUT_RESET_MS) {
                // Set all motor velocities to zero
                set_commands_default();
                set_motor_velocities();
                report_error(NO_COMMANDS_ERROR);
            }
        }
        // Else, leave the velocities as they are.
    }

    // Read encoder counts from SPI if enough time has elapsed since the
    // last successful read
    if ((millis() - last_encoder_msg_recv_ms) > (1000 / ENCODER_READ_RATE_HZ)) {
        if (read_encoder_counts()) {
            // If the read is successful, record the read time
            last_encoder_msg_recv_ms = millis();

            // Send encoder counts over serial if enough time has elapsed since the
            // last successful send. This data is only sent after successful reads
            // to ensure only new data is sent
            if ((millis() - last_encoder_msg_sent_ms) > (1000 / ENCODER_SEND_RATE_HZ)) {
                if (send_encoder_data()) {
                    last_encoder_msg_sent_ms = millis();
                }
            }
        }
        else {
            report_error(ENCODER_READ_ERROR);
        }
    }
}

// TODO: implement a timeout in case of Serial failure
// Should be short, otherwise the rover will become pretty unresponsive
bool read_commands() {
    uint8_t rx_buffer[SERIAL_RX_BUFF_LEN];

    uint8_t read_attempts = 0;

    // Read until we reach the start of the message
    while (Serial.read() != BEGIN_MESSAGE_BYTE) {
        // Unless we don't find it soon enough
        if (read_attempts++ > MAX_COMMAND_READ_ATTEMPTS) {
            // If we don't find it, then abandon the message after
            // reporting the failure
            report_error(COMMAND_FIND_START_ERROR);
            return false;
        }
    }

    // Read message_length of message, not including checksum
    uint8_t message_length = Serial.read();

    // Initialise checksum with message_length
    uint8_t checksum = message_length;

    // Read each byte into the buffer
    for (uint8_t b = 0; b < message_length; b++) {
        rx_buffer[b] = Serial.read();
        // And calculate the checksum as we go
        checksum ^= rx_buffer[b];
    }

    // If the checksum was correct
    uint8_t expected_checksum = Serial.read();
    if (checksum == expected_checksum) {
        // Then copy the data across to the command struct
        memcpy(command_struct_addr, rx_buffer, message_length);
        // And return true to indicate success
        return true;
    }
    // Else, report the error and return false to indicate read failure
    else {
        report_error(COMMAND_CHECKSUM_ERROR);
        return false;
    }
}

void report_error(uint8_t error_code) {
    // If there isn't room to write the error, just ignore it
    if (Serial.availableForWrite() < 2) {
        return;
    }
    Serial.write(BOARD_STATUS_BYTE);
    Serial.write(error_code);
}

void report_data(uint8_t * data, uint8_t len) {
    if (Serial.availableForWrite() < len + 2) {
        return;
    }
    Serial.write(DEBUG_BYTE);
    Serial.write(len);
    Serial.write(data, len);
}

bool read_encoder_counts() {
    uint8_t attempts = 0;

    // Enable the encoder counter SPI slave
    digitalWrite(SS, LOW);

    // Limit attempts so it doesn't lock up if disconnected
    while (attempts++ < SPI_MAX_READ_ATTEMPTS) {
        // Begin SPI communication
        SPI.transfer(REQUEST_ENCODERS);

        // Delays allow slave time to process
        delayMicroseconds(REQUEST_DELAY_US);

        // Read struct length
        uint8_t recv_struct_len = SPI.transfer(SEND_MESSAGE);
        delayMicroseconds(SEND_DELAY_US);  // Same as above

        // Check received struct length is as expected
        if (recv_struct_len != encoder_struct_len) {
            report_error(ENCODER_STRUCT_LEN_MISMATCH);
            if (DEBUG_MODE) {
                uint8_t data[] = {recv_struct_len, encoder_struct_len};
                report_data(data, sizeof(data));
            }
        }
        // Check received struct length isn't too long for the buffer
        else if (recv_struct_len > SPI_RX_BUFF_LEN) {
            report_error(ENCODER_STRUCT_TOO_LONG);
        }
        // If both checks pass, read data. This could be refactored
        // to a separate function
        else {
            // Buffer used so that we don't copy bad data into the
            // encoder counts struct
            uint8_t rx_buffer[SPI_RX_BUFF_LEN];

            // Checksum initialised with message length
            uint8_t checksum = encoder_struct_len;

            // Read in each byte of the struct
            for (uint8_t i = 0; i < encoder_struct_len; i++) {
                rx_buffer[i] = SPI.transfer(SEND_MESSAGE);
                delayMicroseconds(SEND_DELAY_US);  // Delay for slave

                // Calculate checksum as we go
                checksum ^= rx_buffer[i];
            }

            // Read checksum according to slave
            uint8_t recv_checksum = SPI.transfer(END_MESSAGE);
            // If checksum is as expected
            if (checksum == recv_checksum) {
                // Copy data from buffer into encoder struct
                memcpy(encoder_struct_addr, rx_buffer, encoder_struct_len);
                // Insert the timestamp
                encoder_counts_struct.tick_stamp_ms = millis() - enc_count_handshake_time_ms;
                // Disable the encoder counter SPI slave
                digitalWrite(SS, HIGH);
                // And return true to indicate successful transfer
                return true;
            }
            // If checksum is wrong, report the error
            else {
                report_error(ENCODER_CHECKSUM_ERROR);
            }
        }
    }
    digitalWrite(SS, HIGH);

    // If we fail to successfully read a message in the given number of attempts,
    // return false to indicate failed transfer
    return false;
}

void set_motor_velocities() {
    // First set rover wheel velocites, linear then angular
    float rover_target_velocity[2];
    rover_target_velocity[0] = (rover_command_struct.rover_linear_velocity - 100) / 100.0;
    rover_target_velocity[1] = (rover_command_struct.rover_angular_velocity - 100) / 100.0;

    float wheel_speeds[2];

    get_control_outputs(wheel_speeds, rover_target_velocity);
    set_wheel_speeds(wheel_speeds);

    // Then set arm velocities
    set_arm_velocities();
}

void set_servo_positions() {
    // Need to convert from desired position in degrees to required
    // pwm parameters
    uint16_t yaw_tick   = map(rover_command_struct.yaw_servo_position_deg,
                              0, 180,
                              YAW_SERVO_MIN_TICK, YAW_SERVO_MAX_TICK);
    uint16_t pitch_tick = map(rover_command_struct.pitch_servo_position_deg,
                              0, 180,
                              PITCH_SERVO_MIN_TICK, PITCH_SERVO_MAX_TICK);

    servo_pwm.setPWM(CAMERA_YAW_PWM, 0, yaw_tick);
    servo_pwm.setPWM(CAMERA_PITCH_PWM, 0, pitch_tick);
}

// Given the desired (normalised) rover target velocity, determine the required
// (normalised) velocity for each set of wheels
void get_control_outputs(float control_outputs[], float rover_target_velocity[]) {
    // control_outputs returns the desired motor speed for left and right sides
    // rover_target_velocity should have a linear and an angular velocity component

    // Left wheel velocity is difference of linear and angular velocities
    control_outputs[0] = -1 * (rover_target_velocity[0] - rover_target_velocity[1]);

    // Right wheel velocity is sum of linear and angular velocities
    control_outputs[1] = rover_target_velocity[0] + rover_target_velocity[1];

    // Scale outputs to be within the stated limits
    control_outputs[0] *= MOTOR_MAX_SPEED;
    control_outputs[1] *= MOTOR_MAX_SPEED;
}

void set_wheel_speeds(float wheel_speeds[]) {
    // wheel_speeds is a float array: {left_wheels_relative_speed, right_wheels_relative_speed}
    uint8_t wheel_num;
    float duty_cycle, target_wheel_speed;

    for (wheel_num = 0; wheel_num < WHEELS_PER_SIDE; wheel_num++) {
        // Left wheel first
        target_wheel_speed = wheel_speeds[0] * L_MOTOR_RELATIVE_SPEEDS[wheel_num] * MOTOR_MAX_SPEED;
        duty_cycle = 0.5 * (1 + target_wheel_speed);
        set_pwm_duty_cycle(L_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);

        // Then right wheel
        target_wheel_speed = wheel_speeds[1] * R_MOTOR_RELATIVE_SPEEDS[wheel_num] * MOTOR_MAX_SPEED;
        duty_cycle = 0.5 * (1 + target_wheel_speed);
        set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);
    }
}

// TODO I'm sure this can be neatened up
void set_arm_velocities() {
    float duty_cycle, target_speed;
    
    target_speed = -1 * (rover_command_struct.base_rotation_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(BASE_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = (rover_command_struct.arm_actuator_1_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_1_PWM, duty_cycle);

    target_speed = (rover_command_struct.arm_actuator_2_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_2_PWM, duty_cycle);

    target_speed = (rover_command_struct.wrist_rotation_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = (rover_command_struct.wrist_actuator_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ACTUATOR_PWM, duty_cycle);

    target_speed = (rover_command_struct.gripper_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(GRIPPER_MOTOR_PWM, duty_cycle);
}

void set_pwm_duty_cycle(uint8_t pwm_num, float duty_cycle) {
    motor_pwm.setPWM(pwm_num, 0, duty_cycle * (PWM_TICKS - 1));
}

bool send_encoder_data() {
    // Check if there is space in the buffer to write
    if (Serial.availableForWrite() < encoder_struct_len + 3) {
        // Indicate failure if there isn't
        return false;
    }

    // Indicate that we are sending encoder data back
    Serial.write(ENCODER_DATA_BYTE);

    // Initialise the checksum with the message length
    uint8_t struct_len = sizeof(encoder_counts_struct);
    uint8_t checksum = struct_len;

    // Send the message length
    Serial.write(struct_len);

    // Write the whole struct to serial, calculating checksum as we go
    for (uint8_t b = 0; b < struct_len; b++) {
        Serial.write(*(encoder_struct_addr + b));
        checksum ^= *(encoder_struct_addr + b);
    }

    // Then write the checksum
    Serial.write(checksum);

    return true;
}

void set_commands_default() {
    rover_command_struct.rover_linear_velocity = 100;
    rover_command_struct.rover_angular_velocity = 100;
    rover_command_struct.base_rotation_velocity = 100;
    rover_command_struct.arm_actuator_1_velocity = 100;
    rover_command_struct.arm_actuator_2_velocity = 100;
    rover_command_struct.wrist_rotation_velocity = 100;
    rover_command_struct.wrist_actuator_velocity = 100;
    rover_command_struct.gripper_velocity = 100;
    rover_command_struct.yaw_servo_position_deg = 90;
    rover_command_struct.pitch_servo_position_deg = 90;
}

