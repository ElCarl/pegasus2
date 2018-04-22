#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

// Compilation-affecting defs
#define CCC_ENABLED 0  // To enable cross-coupled control. 0 disable, 1 enable

// CONSTANTS

// Setup constants
const bool DEBUG_MODE = 0;

// Physical & electrical constants
const float ROVER_WHEEL_SEPARATION_M = 0.7;  // Could be replaced with half the separation for convenience?
const float WHEEL_CIRCUMFERENCE_M = 0.6283;
const uint8_t WHEELS_PER_SIDE = 3;
const float ROVER_MAX_SPEED_MPS = 1.2;  // Rover max speed, used to convert controller input to actual speed
const float MOTOR_MAX_SPEED = 0.8;  // Normalised - so 0.8 is 80% duty cycle rather than 0.8 m/s
const float L_MOTOR_RELATIVE_SPEEDS[] = {1, 1, 1};  // {F,M,R}. Some motors may be slower than
const float R_MOTOR_RELATIVE_SPEEDS[] = {1, 1, 1};  // others: this allows addressing this

// Encoder constants
const uint8_t N_ENCS = 9;
const uint16_t TICKS_PER_WHEEL_REV = 10 * 48;  // Double-check this. 10 per motor turn, then
const uint8_t FL_ENCODER = 0;                 // 48 per wheel turn
const uint8_t ML_ENCODER = 1;
const uint8_t RL_ENCODER = 2;
const uint8_t FR_ENCODER = 3;
const uint8_t MR_ENCODER = 4;
const uint8_t RR_ENCODER = 5;

// PWM constants
const uint16_t PWM_BOARD_FREQUENCY = 1600;  // Max supported freq of current PWM board is 1600 Hz - unfortunately makes an annoying noise!
const uint32_t PWM_I2C_CLOCKSPEED = 400000UL;    // I2C "fast" mode @ 400 kHz
const uint8_t PWM_CHANNELS = 16;
const uint16_t PWM_TICKS = 4096;
const uint8_t L_FRONT_MOTOR_PWM      = 0;
const uint8_t L_MID_MOTOR_PWM        = 2;
const uint8_t L_REAR_MOTOR_PWM       = 4;
const uint8_t R_FRONT_MOTOR_PWM      = 1;
const uint8_t R_MID_MOTOR_PWM        = 3;
const uint8_t R_REAR_MOTOR_PWM       = 5;
const uint8_t BASE_ROTATE_MOTOR_PWM  = 6;
const uint8_t ARM_ACTUATOR_1_PWM     = 7;
const uint8_t ARM_ACTUATOR_2_PWM     = 8;
const uint8_t GRIPPER_MOTOR_PWM      = 9;
const uint8_t WRIST_ROTATE_MOTOR_PWM = 10;
const uint8_t WRIST_ACTUATOR_PWM     = 11;
const uint8_t L_MOTOR_PWM_CHANNELS[] = {L_FRONT_MOTOR_PWM, L_MID_MOTOR_PWM, L_REAR_MOTOR_PWM};
const uint8_t R_MOTOR_PWM_CHANNELS[] = {R_FRONT_MOTOR_PWM, R_MID_MOTOR_PWM, R_REAR_MOTOR_PWM};

// Serial constants
const uint32_t BRAS_BAUDRATE = 38400UL;  // For comms with the Braswell chip (arduino_communicator_node)
const uint32_t ENC_BAUDRATE  = 38400UL;  // For comms with encoder counter Uno
const uint8_t SOFTSERIAL_RX_PIN = 2;
const uint8_t SOFTSERIAL_TX_PIN = 3;
const byte BOARD_STATUS_BYTE  = 249;
const byte ENCODER_DATA_BYTE  = 250;
const byte SERIAL_READY_BYTE  = 251;
const byte BEGIN_MESSAGE_BYTE = 252;
const byte ARM_MESSAGE_BYTE   = 253;
const byte DRIVE_MESSAGE_BYTE = 254;
const byte END_MESSAGE_BYTE   = 255;
const uint8_t RX_BUFF_LEN     = 64;
const uint8_t MAX_ENCODER_READ_ATTEMPTS = 64;
const uint8_t MAX_COMMAND_READ_ATTEMPTS = 64;
const uint16_t COMMAND_TIMEOUT_RESET_MS = 5000;

// PID constants
const uint8_t ENCODER_HISTORY_LENGTH = 5;
const float WHEEL_KP = 1;
const float WHEEL_KI = 0.1;
const float WHEEL_KD = 0;
const float CCC_KP = 1;
const float CCC_KI = 0.1;
const uint16_t PID_MAX = 4095;  // Currently chosen to simplify translation to setting the PWM
const uint16_t PID_SAMPLE_TIME_MS = 100;

// Other IO constants
const uint8_t MOTOR_ENABLE_PIN = 4;  // Pin chosen at random, change as appropriate

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

// Struct info
uint8_t encoder_struct_len = sizeof(encoder_counts_struct);
uint8_t command_struct_len = sizeof(rover_command_struct);

// Encoder stuff
int32_t lw_encoder_counts[WHEELS_PER_SIDE][ENCODER_HISTORY_LENGTH];
int32_t rw_encoder_counts[WHEELS_PER_SIDE][ENCODER_HISTORY_LENGTH];
int16_t lw_encoder_diffs[WHEELS_PER_SIDE][ENCODER_HISTORY_LENGTH];  // velocity
int16_t rw_encoder_diffs[WHEELS_PER_SIDE][ENCODER_HISTORY_LENGTH];  // velocity
int32_t encoder_times[ENCODER_HISTORY_LENGTH];
int16_t encoder_time_diffs[ENCODER_HISTORY_LENGTH];

// Wheel velocities
// TODO: Check program SRAM - if it's close, change library to use float not double, if there's a difference
double lw_est_vels[WHEELS_PER_SIDE][ENCODER_HISTORY_LENGTH];  // These arrays are rolling averages
double rw_est_vels[WHEELS_PER_SIDE][ENCODER_HISTORY_LENGTH];
double lw_avg_vels[WHEELS_PER_SIDE];
double rw_avg_vels[WHEELS_PER_SIDE];
double lw_output[WHEELS_PER_SIDE];  // Output for each wheel. This is what
double rw_output[WHEELS_PER_SIDE];  // is actually sent to the motors
double lw_desired_vels[WHEELS_PER_SIDE];
double rw_desired_vels[WHEELS_PER_SIDE];

// PID variables
double left_wheels_desired_vel  = 0;
double right_wheels_desired_vel = 0;
double ccc_left_errors[WHEELS_PER_SIDE];
double ccc_right_errors[WHEELS_PER_SIDE];

// PID Controllers
#if CCC_ENABLED
    PID fl_pid(&lw_est_vels[0], &lw_output[0], &lw_desired_vels[0], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID ml_pid(&lw_est_vels[1], &lw_output[1], &lw_desired_vels[1], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rl_pid(&lw_est_vels[2], &lw_output[2], &lw_desired_vels[2], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID fr_pid(&rw_est_vels[0], &rw_output[0], &rw_desired_vels[0], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID mr_pid(&rw_est_vels[1], &rw_output[1], &rw_desired_vels[1], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rr_pid(&rw_est_vels[2], &rw_output[2], &rw_desired_vels[2], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);

    PID ccc_fl(&ccc_left_errors[0], &lw_desired_vels[0], lws_desired_vel);
    PID ccc_ml(&ccc_left_errors[1], &lw_desired_vels[1], lws_desired_vel);
    PID ccc_rl(&ccc_left_errors[2], &lw_desired_vels[2], lws_desired_vel);
    PID ccc_fr(&ccc_right_errors[0], &rw_desired_vels[0], rws_desired_vel);
    PID ccc_mr(&ccc_right_errors[1], &rw_desired_vels[1], rws_desired_vel);
    PID ccc_rr(&ccc_right_errors[2], &rw_desired_vels[2], rws_desired_vel);
#else
    PID fl_pid(&lw_est_vels[0], &lw_output[0], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID ml_pid(&lw_est_vels[1], &lw_output[1], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rl_pid(&lw_est_vels[2], &lw_output[2], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID fr_pid(&rw_est_vels[0], &rw_output[0], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID mr_pid(&rw_est_vels[1], &rw_output[1], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rr_pid(&rw_est_vels[2], &rw_output[2], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
#endif

// SoftwareSerial
SoftwareSerial soft_serial(SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);

// Timings
uint32_t last_command_time_ms = 0;


// MAIN PROGRAM CODE

// Initialisation code

void setup() {
    // Pin modes
    pinMode(MOTOR_ENABLE_PIN, OUTPUT);
    
    // While setting up, ensure motors cannot be used
    disable_motors();

    // Begin PWM
    pwm.begin();
    pwm.setPWMFreq(PWM_BOARD_FREQUENCY);
    Wire.setClock(PWM_I2C_CLOCKSPEED);  // Sets the I2C bus speed

    // Start PWM with all motors stationary
    init_pwm();

    // Begin serial connection to Braswell
    Serial.begin(BRAS_BAUDRATE);

    // Wait until Braswell serial is connected before connecting to encoder counter
    while(!Serial);

    // Begin serial connection to encoder counter
    soft_serial.begin(ENC_BAUDRATE);

    // Send the serial ready byte to indicate readiness for data while awaiting
    // readiness confirmation from Braswell chip
    Serial.write(SERIAL_READY_BYTE);
    while (Serial.read() != SERIAL_READY_BYTE) {
        Serial.write(SERIAL_READY_BYTE);
        delay(100);
    }
    Serial.write(END_MESSAGE_BYTE);
    
    enc_count_handshake_time_ms = millis();
    
    setup_all_pids();

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
    for (uint8_t pwm_num = 0; pwm_num < PWM_CHANNELS; pwm_num++) {
        pwm.setPWM(pwm_num, 0, PWM_TICKS / 2);  // Sets PWM to 50% which stops motors
    }
}

void setup_all_pids() {
    #if CCC_ENABLED
        ccc_fl.SetOutputLimits(0, ROVER_MAX_SPEED_MPS);
        ccc_ml.SetOutputLimits(0, ROVER_MAX_SPEED_MPS);
        ccc_rl.SetOutputLimits(0, ROVER_MAX_SPEED_MPS);
        ccc_fr.SetOutputLimits(0, ROVER_MAX_SPEED_MPS);
        ccc_mr.SetOutputLimits(0, ROVER_MAX_SPEED_MPS);
        ccc_rr.SetOutputLimits(0, ROVER_MAX_SPEED_MPS);

        ccc_fl.SetMode(AUTOMATIC);
        ccc_ml.SetMode(AUTOMATIC);
        ccc_rl.SetMode(AUTOMATIC);
        ccc_fr.SetMode(AUTOMATIC);
        ccc_mr.SetMode(AUTOMATIC);
        ccc_rr.SetMode(AUTOMATIC);
        
        ccc_fl.SetSampleTime(PID_SAMPLE_TIME_MS);
        ccc_ml.SetSampleTime(PID_SAMPLE_TIME_MS);
        ccc_rl.SetSampleTime(PID_SAMPLE_TIME_MS);
        ccc_fr.SetSampleTime(PID_SAMPLE_TIME_MS);
        ccc_mr.SetSampleTime(PID_SAMPLE_TIME_MS);
        ccc_rr.SetSampleTime(PID_SAMPLE_TIME_MS);

    #endif
    fl_pid.SetOutputLimits(0, PID_MAX);
    ml_pid.SetOutputLimits(0, PID_MAX);
    rl_pid.SetOutputLimits(0, PID_MAX);
    fr_pid.SetOutputLimits(0, PID_MAX);
    mr_pid.SetOutputLimits(0, PID_MAX);
    rr_pid.SetOutputLimits(0, PID_MAX);

    fl_pid.SetMode(AUTOMATIC);
    ml_pid.SetMode(AUTOMATIC);
    rl_pid.SetMode(AUTOMATIC);
    fr_pid.SetMode(AUTOMATIC);
    mr_pid.SetMode(AUTOMATIC);
    rr_pid.SetMode(AUTOMATIC);
    
    fl_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
    ml_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
    rl_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
    fr_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
    mr_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
    rr_pid.SetSampleTime(PID_SAMPLE_TIME_MS);
}


// Loop code

void loop() {
    // If any movement commands have been sent,
    if (Serial.available() >= command_struct_len + 3) {
        // and if reading them is successful,
        if (read_commands()) {
            // then set the motor velocities accordingly.
            set_motor_velocities();
            last_command_time_ms = millis();
        }
        else {
            // If we've not got a valid command recently
            if (millis() > last_command_time_ms + COMMAND_TIMEOUT_RESET_MS) {
                // Set all motor velocities to zero
                zero_all_velocities();
                set_motor_velocities();
            }
        }
        // Else, leave the velocities as they are.
    }

    // If encoder data has been sent (+3 is for BEGIN_MESSAGE_BYTE, struct_len and checksum)
    if (soft_serial.available() >= encoder_struct_len + 3) {
        // and if reading them is successful,
        if (read_encoder_counts()) {
            // then send the encoder data to the Braswell chip.
            send_encoder_data();
        }
        // Else, do not send the data.
    }
}

void loop_pid() {
    // If any movement commands have been sent,
    if (Serial.available() >= command_struct_len + 3) {
        // and if reading them is successful,
        if (read_commands()) {
            // then remember the time of receiving the command
            last_command_time_ms = millis();
        }
        else {
            // If we've not got a valid command recently
            if (millis() > last_command_time_ms + COMMAND_TIMEOUT_RESET_MS) {
                // Set all motor velocities to zero
                zero_all_velocities();
                set_motor_velocities();
            }
        }
        // Else, leave the velocities as they are.
    }

    // If encoder data has been sent (+3 is for BEGIN_MESSAGE_BYTE, struct_len and checksum)
    if (soft_serial.available() >= encoder_struct_len + 3) {
        // and if reading them is successful,
        if (read_encoder_counts()) {
            // then first update wheel velocity estimates
            update_wheel_velocity_estimates();
            // then send the encoder data to the Braswell chip.
            send_encoder_data();
        }
        // Else, do not send the data.
    }
}

// TODO: implement a timeout in case of Serial failure
// Should be short, otherwise the rover will become pretty unresponsive
bool read_commands() {
    uint8_t rx_buffer[RX_BUFF_LEN];

    uint8_t read_attempts = 0;

    // Read until we reach the start of the message
    while (Serial.read() != BEGIN_MESSAGE_BYTE) {
        // Unless we don't find it soon enough
        if (read_attempts++ > MAX_COMMAND_READ_ATTEMPTS) {
            // If we don't, then abandon the message
            return false;
        }
    }

    // Read message_length of message - should not include checksum!
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
        // Copy the data across to the command struct
        memcpy(&rover_command_struct, rx_buffer, message_length);
        // And return true to indicate success
        return true;
    }

    // Else, ignore the message
    // And return false to indicate failure
    if (DEBUG_MODE) {
        Serial.write(BOARD_STATUS_BYTE);
        static byte msg[] = "Checksum incorrect";
        Serial.write(sizeof(msg));
        Serial.write(msg, sizeof(msg));
    }
    return false;
}

// TODO: see above
// TODO: figure out how to abstract the logic here so that the same function can
// be used for both Serial and soft_serial reading
bool read_encoder_counts() {
    uint8_t checksum;
    uint8_t rx_buffer[RX_BUFF_LEN];

    uint8_t attempts = 0;
    uint32_t count_time = millis();

    // Read until we reach the start of the message
    while (soft_serial.read() != BEGIN_MESSAGE_BYTE) {
        // If we don't find the BEGIN_MESSAGE_BYTE in time, abort
        if (attempts++ > MAX_ENCODER_READ_ATTEMPTS) {
            return false;
        }
    }

    // Read message_length of message, not including checksum
    uint8_t message_length = soft_serial.read();

    // Initialise checksum with message_length
    checksum = message_length;

    // Read each byte into the buffer
    for (uint8_t b = 0; b < message_length; b++) {
        rx_buffer[b] = soft_serial.read();
        // And calculate the checksum as we go
        checksum ^= rx_buffer[b];
    }

    // If the checksum is correct
    if (checksum == soft_serial.read()) {
        // Copy in the time at the start of the serial message, assumed to be
        // the same as the encoder time. TODO since we're now using this for control
        // it might be worthwhile ensuring the encoder count timings are accurate...
        encoder_counts_struct.tick_stamp_ms = count_time;
        // Copy the data across to the encoder counts struct
        memcpy(&encoder_counts_struct, rx_buffer, message_length);
        // And return true to indicate success
        return true;
    }
    // Else, ignore the message
    // And return false to indicate failure
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

// Given the desired (normalised) rover target velocity, determine the required
// (normalised) velocity for each set of wheels
void get_control_outputs(float control_outputs[], float rover_target_velocity[]) {
    // control_outputs returns the desired motor speed for left and right sides
    // rover_target_velocity should have a linear and an angular velocity component

    // Left wheel velocity is difference of linear and angular velocities
    control_outputs[0] = -1 * (rover_target_velocity[0] - rover_target_velocity[1]);

    // Right wheel velocity is sum of linear and angular velocities
    control_outputs[1] = rover_target_velocity[0] + rover_target_velocity[1];

    // Find largest absolute control value
    float max_control;
    if (abs(control_outputs[0]) > abs(control_outputs[1])) {
        max_control = abs(control_outputs[0]);
    }
    else {
        max_control = abs(control_outputs[1]);
    }

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

// This whole setup could probably be significantly cleaned up by having a good think
// about what arrays are actually necessary, which need to be held long-term etc. Might
// be easier/more clear to convert everything to metres ASAP.
void update_wheel_velocity_estimates() {
    // Initial update of avg velocities
    float distance_m;
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        distance_m = lw_encoder_diffs[wheel][ENCODER_HISTORY_LENGTH - 1] * (WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV);
        lw_avg_vels[wheel] -= distance_m / encoder_time_diffs[ENCODER_HISTORY_LENGTH - 1];

        distance_m = rw_encoder_diffs[wheel][ENCODER_HISTORY_LENGTH - 1] * (WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV);
        rw_avg_vels[wheel] -= distance_m / encoder_time_diffs[ENCODER_HISTORY_LENGTH - 1];
    }
    
    // Make room for the new values
    for (uint8_t i = ENCODER_HISTORY_LENGTH - 1; i > 0; i--) {
        encoder_times[i] = encoder_times[i - 1];
        encoder_time_diffs[i] = encoder_time_diffs[i - 1];
    }
    // Do the same for the encoder counts
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        for (uint8_t i = ENCODER_HISTORY_LENGTH - 1; i > 0; i--) {
            lw_encoder_counts[wheel][i] = lw_encoder_counts[wheel][i - 1];
            rw_encoder_counts[wheel][i] = rw_encoder_counts[wheel][i - 1];
            lw_encoder_diffs[wheel][i] = lw_encoder_diffs[wheel][i - 1];
            rw_encoder_diffs[wheel][i] = rw_encoder_diffs[wheel][i - 1];
        }
    }
    // Put the new values in
    encoder_times[0] = encoder_counts_struct.tick_stamp_ms;
    encoder_time_diffs[0] = encoder_times[0] - encoder_times[1];
    lw_encoder_counts[0][0] = encoder_counts_struct.encoder_counts[FL_ENCODER];
    lw_encoder_counts[1][0] = encoder_counts_struct.encoder_counts[ML_ENCODER];
    lw_encoder_counts[2][0] = encoder_counts_struct.encoder_counts[RL_ENCODER];
    rw_encoder_counts[0][0] = encoder_counts_struct.encoder_counts[FR_ENCODER];
    rw_encoder_counts[1][0] = encoder_counts_struct.encoder_counts[MR_ENCODER];
    rw_encoder_counts[2][0] = encoder_counts_struct.encoder_counts[RR_ENCODER];
    
    // New diffs
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        lw_encoder_diffs[wheel][0] = lw_encoder_counts[wheel][0] - lw_encoder_counts[wheel][1];
        rw_encoder_diffs[wheel][0] = rw_encoder_counts[wheel][0] - rw_encoder_counts[wheel][1];
    }
    
    // Final avg vel update
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        distance_m = lw_encoder_diffs[wheel][0] * (WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV);
        lw_avg_vels[wheel] += distance_m / encoder_time_diffs[0];
        distance_m = rw_encoder_diffs[wheel][0] * (WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV);
        rw_avg_vels[wheel] += distance_m / encoder_time_diffs[0];
    }
}

void send_encoder_data() {
    Serial.write(ENCODER_DATA_BYTE);

    uint8_t struct_len = sizeof(encoder_counts_struct);

    // Set the timestamp
    encoder_counts_struct.tick_stamp_ms = millis();

    // Initialise the checksum with the message length
    uint8_t checksum = struct_len;

    // Send the message length
    Serial.write(struct_len);

    // Write the whole struct to serial, calculating checksum as we go
    uint8_t * struct_ptr = (uint8_t *)&encoder_counts_struct;
    for (uint8_t b = 0; b < struct_len; b++) {
        Serial.write(*(struct_ptr + b));
        checksum ^= *(struct_ptr + b);
    }

    // Then write the checksum
    Serial.write(checksum);
}

void zero_all_velocities() {
    rover_command_struct.rover_linear_velocity = 100;
    rover_command_struct.rover_angular_velocity = 100;
    rover_command_struct.base_rotation_velocity = 100;
    rover_command_struct.arm_actuator_1_velocity = 100;
    rover_command_struct.arm_actuator_2_velocity = 100;
    rover_command_struct.wrist_rotation_velocity = 100;
    rover_command_struct.wrist_actuator_velocity = 100;
    rover_command_struct.gripper_velocity = 100;
}

