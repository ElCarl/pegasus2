#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <SPI.h>

// Compilation-affecting defs
#define CCC_ENABLED 0  // To enable cross-coupled control. 0 disable, 1 enable

// CONSTANTS

// Debug constants
const bool DEBUG_MODE = 1;

// Physical & electrical constants
const float ROVER_HALF_WHEEL_SEP_M = 0.7;  // Could be replaced with half the separation for convenience?
const float WHEEL_CIRCUMFERENCE_M = 0.6283;
const uint8_t WHEELS_PER_SIDE = 3;
const float ROVER_MAX_SPEED_MPS = 1.2;  // Rover max speed, used to convert controller input to actual speed
const float MOTOR_MAX_SPEED = 0.8;  // Normalised - so 0.8 is 80% duty cycle rather than 0.8 m/s
const float L_MOTOR_RELATIVE_SPEEDS[] = {1, 1, 1};  // {F,M,R}. Some motors may be slower than
const float R_MOTOR_RELATIVE_SPEEDS[] = {1, 1, 1};  // others: this allows addressing this
const uint8_t MOTOR_GEAR_RATIO = 24; // Motor gearbox ratio - check
const uint8_t WHEEL_GEAR_RATIO = 2;  // Motor output to wheel rotations


// Encoder constants
const uint8_t N_ENCS = 7;  // 3 wheels each side & the gripper
const uint8_t FL_ENCODER = 4;
const uint8_t ML_ENCODER = 5;
const uint8_t RL_ENCODER = 6;
const uint8_t FR_ENCODER = 2;
const uint8_t MR_ENCODER = 1;
const uint8_t RR_ENCODER = 0;
const uint8_t ENCODER_READ_RATE_HZ = 20;
const uint8_t ENCODER_SEND_RATE_HZ = 5;  // Encodometry probably needs less data than PID
const uint8_t TICKS_PER_MOTOR_REV = 10;  // Check
const uint16_t TICKS_PER_WHEEL_REV = TICKS_PER_MOTOR_REV * MOTOR_GEAR_RATIO * WHEEL_GEAR_RATIO;

// PWM constants
const unsigned int PWM_BOARD_FREQUENCY = 1600;  // Max supported freq of current PWM board is ~1600 Hz - unfortunately makes an annoying noise!
const unsigned long PWM_I2C_CLOCKSPEED = 400000;    // I2C fast mode @ 400 kHz
const uint8_t L_FRONT_MOTOR_PWM        = 1;
const uint8_t L_MID_MOTOR_PWM          = 3;
const uint8_t L_REAR_MOTOR_PWM         = 5;
const uint8_t R_FRONT_MOTOR_PWM        = 0;
const uint8_t R_MID_MOTOR_PWM          = 2;
const uint8_t R_REAR_MOTOR_PWM         = 4;
const uint8_t BASE_ROTATE_MOTOR_PWM    = 11;
const uint8_t ARM_ACTUATOR_1_PWM       = 7;
const uint8_t ARM_ACTUATOR_2_PWM       = 8;
const uint8_t GRIPPER_MOTOR_PWM        = 9;
const uint8_t WRIST_ROTATE_MOTOR_PWM   = 10;
const uint8_t WRIST_ACTUATOR_PWM       = 6;
const uint8_t L_MOTOR_PWM_CHANNELS[]   = {L_FRONT_MOTOR_PWM, L_MID_MOTOR_PWM, L_REAR_MOTOR_PWM};
const uint8_t R_MOTOR_PWM_CHANNELS[]   = {R_FRONT_MOTOR_PWM, R_MID_MOTOR_PWM, R_REAR_MOTOR_PWM};
const uint8_t PWM_CHANNELS = 16;
const uint16_t PWM_TICKS = 4096;

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
const uint8_t RX_BUFF_LEN     = 64;
const uint8_t SERIAL_RX_BUFF_LEN = 64;
const uint8_t MAX_COMMAND_READ_ATTEMPTS = 64;
const uint16_t COMMAND_TIMEOUT_RESET_MS = 5000;

// PID constants
const uint8_t ENCODER_HISTORY_LENGTH = 5;
const float WHEEL_KP = 1;
const float WHEEL_KI = 0.1;
const float WHEEL_KD = 0;
const float CCC_KP = 1;
const float CCC_KI = 0.1;
const uint16_t PID_SAMPLE_TIME_MS = 50;

// SPI constants
const uint32_t SPI_CLOCKSPEED_HZ = 4000000;
const uint8_t REQUEST_DELAY_US = 75;
const uint8_t SEND_DELAY_US    = 9;
const uint8_t REQUEST_ENCODERS = 248;
const uint8_t SEND_MESSAGE     = 252;
const uint8_t END_MESSAGE      = 255;
const uint8_t SPI_MAX_READ_ATTEMPTS = 3;
const uint8_t SPI_RX_BUFF_LEN = 64;

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
    uint8_t lin_vel;
    uint8_t ang_vel;
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
ROVER_COMMAND_DATA_STRUCTURE commands;

// Struct info
uint8_t * encoder_struct_addr = (uint8_t *)&encoder_counts_struct;
uint8_t * command_struct_addr = (uint8_t *)&commands;
uint8_t encoder_struct_len = sizeof(encoder_counts_struct);
uint8_t command_struct_len = sizeof(commands);

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
    PID fl_pid(&lw_est_vels[0][0], &lw_output[0], &lw_desired_vels[0], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID ml_pid(&lw_est_vels[1][0], &lw_output[1], &lw_desired_vels[1], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rl_pid(&lw_est_vels[2][0], &lw_output[2], &lw_desired_vels[2], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID fr_pid(&rw_est_vels[0][0], &rw_output[0], &rw_desired_vels[0], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID mr_pid(&rw_est_vels[1][0], &rw_output[1], &rw_desired_vels[1], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rr_pid(&rw_est_vels[2][0], &rw_output[2], &rw_desired_vels[2], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);

    PID ccc_fl(&ccc_left_errors[0], &lw_desired_vels[0], lws_desired_vel);
    PID ccc_ml(&ccc_left_errors[1], &lw_desired_vels[1], lws_desired_vel);
    PID ccc_rl(&ccc_left_errors[2], &lw_desired_vels[2], lws_desired_vel);
    PID ccc_fr(&ccc_right_errors[0], &rw_desired_vels[0], rws_desired_vel);
    PID ccc_mr(&ccc_right_errors[1], &rw_desired_vels[1], rws_desired_vel);
    PID ccc_rr(&ccc_right_errors[2], &rw_desired_vels[2], rws_desired_vel);
#else
    PID fl_pid(&lw_est_vels[0][0], &lw_output[0], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID ml_pid(&lw_est_vels[1][0], &lw_output[1], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rl_pid(&lw_est_vels[2][0], &lw_output[2], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID fr_pid(&rw_est_vels[0][0], &rw_output[0], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID mr_pid(&rw_est_vels[1][0], &rw_output[1], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rr_pid(&rw_est_vels[2][0], &rw_output[2], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
#endif

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
    init_pid_arrays();

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

    fl_pid.SetOutputLimits(0, 1);
    ml_pid.SetOutputLimits(0, 1);
    rl_pid.SetOutputLimits(0, 1);
    fr_pid.SetOutputLimits(0, 1);
    mr_pid.SetOutputLimits(0, 1);
    rr_pid.SetOutputLimits(0, 1);

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

// Sets PID related arrays to zero initially. May have issues on boot if encoder
// counter is reporting nonzero values as it will assume a very high velocity.
// Probably easier to fix this by just waiting a moment after booting!
void init_pid_arrays() {
    for (uint8_t i = 0; i < WHEELS_PER_SIDE; i++) {
        for (uint8_t j =0; j < ENCODER_HISTORY_LENGTH; j++) {
            lw_est_vels[i][j] = 0;
            rw_est_vels[i][j] = 0;
            lw_encoder_counts[i][j] = 0;
            rw_encoder_counts[i][j] = 0;
            lw_encoder_diffs[i][j] = 0;
            rw_encoder_diffs[i][j] = 0;
        }

        lw_avg_vels[i] = 0;
        rw_avg_vels[i] = 0;
        lw_output[i] = 0;
        rw_output[i] = 0;

        #if CCC_ENABLED
            lw_desired_vels[i] = 0;
            rw_desired_vels[i] = 0;
            ccc_left_errors[i] = 0;
            ccc_right_errors[i] = 0;
        #endif
    }

    for (uint8_t j = 0; j < ENCODER_HISTORY_LENGTH; j++) {
        encoder_times[j] = 0;
        encoder_time_diffs[j] = 0;
    }
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
            // then set the motor and arm velocities accordingly.
            set_motor_velocities();
            set_arm_velocities();

            last_command_time_ms = millis();
        }
        else {
            // If we've not got a valid command recently
            if (millis() > last_command_time_ms + COMMAND_TIMEOUT_RESET_MS) {
                // Set all motor velocities to zero
                zero_all_velocities();
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

            update_wheel_velocity_estimates();

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
    uint32_t count_time = millis();

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
                encoder_counts_struct.tick_stamp_ms = count_time - enc_count_handshake_time_ms;
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

    // Disable SPI slave
    digitalWrite(SS, HIGH);

    // If we fail to successfully read a message in the given number of attempts,
    // return false to indicate failed transfer
    return false;
}

void set_motor_velocities() {
    float = 

    left_wheels_desired_vel = commands.lin_vel + (ROVER_HALF_WHEEL_SEP_M * commands.ang_vel);
    right_wheels_desired_vel = commands.lin_vel - (ROVER_HALF_WHEEL_SEP_M * commands.ang_vel);

    
}

//void set_motor_velocities() {
//    // First set rover wheel velocites, linear then angular
//    float rover_target_velocity[2];
//    rover_target_velocity[0] = (commands.lin_vel - 100) / 100.0;
//    rover_target_velocity[1] = (commands.ang_vel - 100) / 100.0;
//
//    float wheel_speeds[2];
//
//    get_control_outputs(wheel_speeds, rover_target_velocity);
//    set_wheel_speeds(wheel_speeds);
//}
//
//// Given the desired (normalised) rover target velocity, determine the required
//// (normalised) velocity for each set of wheels
//void get_control_outputs(float control_outputs[], float rover_target_velocity[]) {
//    // control_outputs returns the desired motor speed for left and right sides
//    // rover_target_velocity should have a linear and an angular velocity component
//
//    // Left wheel velocity is difference of linear and angular velocities
//    control_outputs[0] = -1 * (rover_target_velocity[0] - rover_target_velocity[1]);
//
//    // Right wheel velocity is sum of linear and angular velocities
//    control_outputs[1] = rover_target_velocity[0] + rover_target_velocity[1];
//
//    // Scale outputs to be within the stated limits
//    control_outputs[0] *= MOTOR_MAX_SPEED;
//    control_outputs[1] *= MOTOR_MAX_SPEED;
//}
//
//void set_wheel_speeds(float wheel_speeds[]) {
//    // wheel_speeds is a float array: {left_wheels_relative_speed, right_wheels_relative_speed}
//    uint8_t wheel_num;
//    float duty_cycle, target_wheel_speed;
//
//    for (wheel_num = 0; wheel_num < WHEELS_PER_SIDE; wheel_num++) {
//        // Left wheel first
//        target_wheel_speed = wheel_speeds[0] * L_MOTOR_RELATIVE_SPEEDS[wheel_num] * MOTOR_MAX_SPEED;
//        duty_cycle = 0.5 * (1 + target_wheel_speed);
//        set_pwm_duty_cycle(L_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);
//
//        // Then right wheel
//        target_wheel_speed = wheel_speeds[1] * R_MOTOR_RELATIVE_SPEEDS[wheel_num] * MOTOR_MAX_SPEED;
//        duty_cycle = 0.5 * (1 + target_wheel_speed);
//        set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);
//    }
//}

// TODO I'm sure this can be neatened up
void set_arm_velocities() {
    float duty_cycle, target_speed;
    
    target_speed = (commands.base_rotation_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(BASE_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = (commands.arm_actuator_1_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_1_PWM, duty_cycle);

    target_speed = (commands.arm_actuator_2_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_2_PWM, duty_cycle);

    target_speed = (commands.wrist_rotation_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = (commands.wrist_actuator_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ACTUATOR_PWM, duty_cycle);

    target_speed = (commands.gripper_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_SPEED;
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
        distance_m = lw_encoder_diffs[wheel][ENCODER_HISTORY_LENGTH - 1] * WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV;
        lw_avg_vels[wheel] -= distance_m / encoder_time_diffs[ENCODER_HISTORY_LENGTH - 1];

        distance_m = rw_encoder_diffs[wheel][ENCODER_HISTORY_LENGTH - 1] * WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV;
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

bool send_encoder_data() {
    // Check if there is space in the buffer to write
    if (Serial.availableForWrite() < encoder_struct_len + 3) {
        // Indicate failure if there isn't
        return false;
    }

    // Indicate that we are sending encoder data back
    Serial.write(ENCODER_DATA_BYTE);

    // Initialise the checksum with the message length
    uint8_t struct_len = encoder_struct_len;
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

void zero_all_velocities() {
    commands.lin_vel = 100;
    commands.ang_vel = 100;
    commands.base_rotation_velocity = 100;
    commands.arm_actuator_1_velocity = 100;
    commands.arm_actuator_2_velocity = 100;
    commands.wrist_rotation_velocity = 100;
    commands.wrist_actuator_velocity = 100;
    commands.gripper_velocity = 100;
}

