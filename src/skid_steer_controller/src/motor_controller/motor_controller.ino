#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>



// CONSTANTS

// Debug constants
const bool DEBUG_MODE = 0;

// Electrical & physical constants
const float MOTOR_MAX_SPEED = 0.7;  // Normalised - so 0.7 is 70% duty cycle rather than 0.7 m/s
const float L_FRONT_MOTOR_RELATIVE_SPEED = 1;  // Some motors may be slower than others:
const float L_MID_MOTOR_RELATIVE_SPEED   = 1;  // this should offer a way of addressing this
const float L_REAR_MOTOR_RELATIVE_SPEED  = 1;
const float R_FRONT_MOTOR_RELATIVE_SPEED = 1;
const float R_MID_MOTOR_RELATIVE_SPEED   = 1;
const float R_REAR_MOTOR_RELATIVE_SPEED  = 1;
const float L_MOTOR_RELATIVE_SPEEDS[] = {L_FRONT_MOTOR_RELATIVE_SPEED, L_MID_MOTOR_RELATIVE_SPEED, L_REAR_MOTOR_RELATIVE_SPEED};
const float R_MOTOR_RELATIVE_SPEEDS[] = {R_FRONT_MOTOR_RELATIVE_SPEED, R_MID_MOTOR_RELATIVE_SPEED, R_REAR_MOTOR_RELATIVE_SPEED};
const uint8_t WHEELS_PER_SIDE = 3;
const uint8_t N_ENCS = 9;

// PWM constants
const unsigned int  PWM_BOARD_FREQUENCY      = 1600;  // Max supported freq of current PWM board is 1600 Hz - unfortunately makes an annoying noise!
const unsigned long PWM_I2C_CLOCKSPEED = 400000UL;    // I2C "fast" mode @ 400 kHz
const uint8_t L_FRONT_MOTOR_PWM        = 0;
const uint8_t L_MID_MOTOR_PWM          = 1;
const uint8_t L_REAR_MOTOR_PWM         = 2;
const uint8_t R_FRONT_MOTOR_PWM        = 3;
const uint8_t R_MID_MOTOR_PWM          = 4;
const uint8_t R_REAR_MOTOR_PWM         = 5;
const uint8_t BASE_ROTATE_MOTOR_PWM    = 6;
const uint8_t ARM_ACTUATOR_1_PWM       = 7;
const uint8_t ARM_ACTUATOR_2_PWM       = 8;
const uint8_t WRIST_ROTATE_MOTOR_PWM   = 9;
const uint8_t WRIST_ACTUATOR_PWM       = 10;
const uint8_t GRIPPER_MOTOR_PWM        = 11;
const uint8_t L_MOTOR_PWM_CHANNELS[]   = {L_FRONT_MOTOR_PWM, L_MID_MOTOR_PWM, L_REAR_MOTOR_PWM};
const uint8_t R_MOTOR_PWM_CHANNELS[]   = {R_FRONT_MOTOR_PWM, R_MID_MOTOR_PWM, R_REAR_MOTOR_PWM};

// Serial constants
const unsigned long HS_BAUDRATE  = 38400;  // For comms with the Braswell chip (arduino_communicator_node)
const unsigned long BRAS_BAUDRATE = 38400;  // For comms with encoder counter Uno
const uint8_t SOFTSERIAL_RX_PIN   = 2;
const uint8_t SOFTSERIAL_TX_PIN   = 3;
const unsigned long TIMEOUT_MS    = 5000;   // TODO - actually implement this!
const byte ENCODER_DATA_BYTE      = 250;
const byte SERIAL_READY_BYTE      = 251;
const byte BEGIN_MESSAGE_BYTE     = 252;
const byte ARM_MESSAGE_BYTE       = 253;
const byte DRIVE_MESSAGE_BYTE     = 254;
const byte END_MESSAGE_BYTE       = 255;
const uint8_t RX_BUFF_LEN         = 64;

// Other IO constants
const uint8_t MOTOR_ENABLE_PIN = 4;  // Pin chosen at random, change as appropriate
const uint8_t PWM_CHANNELS = 16;
const unsigned int PWM_TICKS = 4096;



// STRUCTS

// Define the RX data structure
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
};



// GLOBAL VARIABLES

// Handshake timings
uint32_t enc_count_handshake_time_ms;
uint32_t handshake_offset_ms;

// PWM board object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Structs
ENCODER_DATA_STRUCTURE encoder_counts_struct;
ROVER_COMMAND_DATA_STRUCTURE rover_command_struct;



// MAIN PROGRAM CODE


// Initialisation code

void setup() {
    // Pin modes
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    
    disable_motors();

    // Begin serial connections
    Serial.begin(HS_BAUDRATE);
    //Serial1.begin(BRAS_BAUDRATE);

    while(!Serial);

    // Begin PWM
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);
    Wire.setClock(PWM_I2C_CLOCKSPEED);  // Sets the I2C bus speed

    // Start PWM with all motors stationary
    init_pwm();
    
    /* Disabled to simplify testing
    // Send the serial ready byte to indicate readiness for data while awaiting
    // readiness confirmation from encoder counter Uno
    Serial1.write(SERIAL_READY_BYTE);
    while (Serial1.read() != SERIAL_READY_BYTE) {
        Serial1.write(SERIAL_READY_BYTE);
        delay(100);
        // May need to implement some way to calibrate for different start times
        // of Arduino boards? Offset in ms between boards?
    }
    */
    
    enc_count_handshake_time_ms = millis();
    
    // Send the serial ready byte to indicate readiness for data while awaiting
    // readiness confirmation from Braswell chip
    Serial.write(SERIAL_READY_BYTE);
    while (Serial.read() != SERIAL_READY_BYTE) {
        Serial.write(SERIAL_READY_BYTE);
        delay(100);
    }
    
    handshake_offset_ms = millis() - enc_count_handshake_time_ms;
    
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
    for (uint8_t pwm_num = 0; pwm_num < PWM_CHANNELS; pwm_num++) {
        pwm.setPWM(pwm_num, 0, PWM_TICKS / 2);  // Sets PWM to 50% which stops motors
    }
}


// Loop code

void loop() {
    // If any movement commands have been sent,
    if (Serial.available() > 0) {
        // and if reading them is successful,
        if (read_commands()) {
            // then set the motor velocities accordingly.
            set_motor_velocities();
        }
        // Else, leave the velocities as they are.
    }

    /*
    // If any encoder data has been sent,
    if (Serial1.available() > 0) {
        // and if reading them is successful,
        if (read_encoder_counts()) {
            // then send the encoder data to the Braswell chip.
            send_encoder_data();
        }
        // Else, do not send the data.
    }*/
}

// TODO: implement a timeout in case of Serial failure
// Should be short, otherwise the rover will become pretty unresponsive
bool read_commands() {
    // Could change this to start equal to size, would be a little more robust
    uint8_t checksum;  
    uint8_t rx_buffer[RX_BUFF_LEN];

    // Read until we reach the start of the message
    while (Serial.read() != BEGIN_MESSAGE_BYTE) {}

    // Check to ensure data is available
    while (Serial.available() == 0) {}

    // Read message_length of message - should not include checksum!
    uint8_t message_length = Serial.read();

    // Initialise checksum with message_length
    checksum = message_length;

    // Read each byte into the buffer
    for (uint8_t b = 0; b < message_length; b++) {
        // Wait until data is available. Is this needed, or does it just slow it down?
        while (Serial.available() == 0) {}
        rx_buffer[b] = Serial.read();
        // And calculate the checksum as we go
        checksum ^= rx_buffer[b];
    }

    // If the checksum was correct
    if (checksum == Serial.read()) {
        // Copy the data across to the command struct
        memcpy(&rover_command_struct, rx_buffer, message_length);
        // Read until the end of the message
        while (Serial.read() != END_MESSAGE_BYTE) {}  // Probably don't need END_MESSAGE_BYTE
        // And return true to indicate success
        if (DEBUG_MODE) { Serial.print("Checksum correct"); }
        return true;
    }
    // Else, ignore the message
    // And return false to indicate failure
    if (DEBUG_MODE) { Serial.print("Checksum incorrect"); }
    return false;

}

// TODO: see above
// TODO: figure out how to abstract the logic here so that the same function can
// be used for both Serial and Serial1 reading
bool read_encoder_counts() {
    uint8_t checksum;
    uint8_t rx_buffer[RX_BUFF_LEN];

    // Read until we reach the start of the message
    while (Serial1.read() != BEGIN_MESSAGE_BYTE) {}

    // Check to ensure data is available
    while (Serial1.available() == 0) {}

    // Read message_length of message, not including checksum
    uint8_t message_length = Serial1.read();

    // Initialise checksum with message_length
    checksum = message_length;

    // Read each byte into the buffer
    for (uint8_t b = 0; b < message_length; b++) {
        // Wait until data is available. Is this needed, or does it just slow it down?
        while (Serial1.available() == 0) {}
        rx_buffer[b] = Serial1.read();
        // And calculate the checksum as we go
        checksum ^= rx_buffer[b];
    }

    // If the checksum is correct
    if (checksum == Serial1.read()) {
        // Copy the data across to the encoder counts struct
        memcpy(&encoder_counts_struct, rx_buffer, message_length);
        // Read until the end of the message
        while (Serial1.read() != END_MESSAGE_BYTE) {}
        // And return true to indicate success
        return true;
    }
    // Else, ignore the message
    // And return false to indicate failure
    return false;
}

void set_motor_velocities() {
    // First set rover wheel velocites
    float rover_target_velocity[2];
    rover_target_velocity[0] = (rover_command_struct.rover_linear_velocity - 100) / 100.0;
    rover_target_velocity[1] = (rover_command_struct.rover_angular_velocity - 100) / 100.0;

    float wheel_speeds[2];

    get_control_outputs(wheel_speeds, rover_target_velocity);
    set_wheel_speeds(wheel_speeds);

    // Then set arm velocities
    set_arm_velocities();
}

// Given the desired (normalised) rover target velocity, determine the required
// (normalised) velocity for each set of wheels
void get_control_outputs(float control_outputs[], float rover_target_velocity[]) {
    // control_outputs returns the desired motor speed for left and right sides
    // rover_target_velocity should have a linear and an angular velocity component

    // Left wheel velocity is sum of linear and angular velocities
    control_outputs[0] = rover_target_velocity[0] + rover_target_velocity[1];

    // Right wheel velocity is the difference between linear and angular velocities
    control_outputs[1] = rover_target_velocity[0] - rover_target_velocity[1];

    // Find largest absolute control value
    float max_control;
    if (abs(control_outputs[0]) > abs(control_outputs[1])) {
        max_control = abs(control_outputs[0]);
    }
    else {
        max_control = abs(control_outputs[1]);
    }

    // Check if output scaling is required
    if (max_control > MOTOR_MAX_SPEED) {
        // If it is, scale to max value of 1
        control_outputs[0] /= max_control;
        control_outputs[1] /= max_control;
        // Then scale to max as defined by MOTOR_MAX_SPEED
        control_outputs[0] *= MOTOR_MAX_SPEED;
        control_outputs[1] *= MOTOR_MAX_SPEED;
    }
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

        // Then right wheel - negative since they turn opposite to the left wheels
        target_wheel_speed = -1 * wheel_speeds[1] * R_MOTOR_RELATIVE_SPEEDS[wheel_num] * MOTOR_MAX_SPEED;
        duty_cycle = 0.5 * (1 + target_wheel_speed);
        set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);
    }
}

void set_arm_velocities() {
    float duty_cycle, target_speed;
    
    target_speed = rover_command_struct.base_rotation_velocity * MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(BASE_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = rover_command_struct.arm_actuator_1_velocity * MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_1_PWM, duty_cycle);

    target_speed = rover_command_struct.arm_actuator_2_velocity * MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_2_PWM, duty_cycle);

    target_speed = rover_command_struct.wrist_rotation_velocity * MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = rover_command_struct.wrist_actuator_velocity * MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ACTUATOR_PWM, duty_cycle);

    target_speed = rover_command_struct.gripper_velocity * MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(GRIPPER_MOTOR_PWM, duty_cycle);
}

void set_pwm_duty_cycle(uint8_t pwm_num, float duty_cycle) {
    pwm.setPWM(pwm_num, 0, duty_cycle * (PWM_TICKS - 1));
}


void send_encoder_data() {
    Serial.write(ENCODER_DATA_BYTE);
    byte buffer[4];

    // Offset the timestamp
    encoder_counts_struct.tick_stamp_ms -= handshake_offset_ms;

    // Initialise the checksum with the message length
    uint8_t checksum = sizeof(encoder_counts_struct);

    // Write the timestamp to serial, LSB to MSB
    long_to_bytes(encoder_counts_struct.tick_stamp_ms, buffer);
    for (uint8_t b = 0; b < 4; b++) {
        Serial.write(buffer[b]);
        // Calculate checksum as we go
        checksum ^= buffer[b];
    }
    // Then each of the encoders, same order
    for (uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
        long_to_bytes(encoder_counts_struct.encoder_counts[encoder], buffer);
        for (uint8_t b = 0; b < 4; b++) {
            Serial.write(buffer[b]);
            // Calculate checksum as we go
            checksum ^= buffer[b];
        }
    }
    // Then the checksum
    Serial.write(checksum);

    // Then end the message
    Serial.write(END_MESSAGE_BYTE);
}

// Converts a long to an array of 4 bytes for serial transmission
void long_to_bytes(int32_t val, byte bytes[]) {
    bytes[0] =  val        & 255;
    bytes[1] = (val >> 8)  & 255;
    bytes[2] = (val >> 16) & 255;
    bytes[3] = (val >> 24) & 255;
}

