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
const float ROVER_HALF_WHEEL_SEP_M = 0.35;  // Could be replaced with half the separation for convenience?
const float WHEEL_CIRCUMFERENCE_M = 0.6283;
const uint8_t WHEELS_PER_SIDE = 3;
const float ROVER_MAX_LIN_SPEED_MPS = 1.2;  // Rover max linear speed, used to convert controller input to actual speed
const float ROVER_MAX_ANG_SPEED_RPS = 0.3;  // Rover max angular speed
const float MOTOR_MAX_DUTY_CYCLE = 0.8;  // Normalised - so 0.8 is 80% duty cycle rather than 0.8 m/s
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
const uint8_t PWM_CHANNELS = 16;
const uint16_t PWM_TICKS = 4096;

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
const uint8_t PID_MAX_ENCODER_FAILURES = 5;
const uint16_t ENCODER_ALLOWABLE_FAILURE_PERIOD_MS = 100;

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
uint32_t last_encoder_failure = 0;
uint8_t recent_encoder_failures = 0;

// Wheel velocities
// TODO: Check program SRAM - if it's close, change library to use float not double, if there's a difference
double lw_avg_vels[WHEELS_PER_SIDE];
double rw_avg_vels[WHEELS_PER_SIDE];
double lw_output[WHEELS_PER_SIDE];  // Output for each wheel. This is what
double rw_output[WHEELS_PER_SIDE];  // is actually sent to the motors
double lw_desired_vels[WHEELS_PER_SIDE];
double rw_desired_vels[WHEELS_PER_SIDE];

// PID variables
bool pid_enabled = true;
double left_wheels_desired_vel  = 0;
double right_wheels_desired_vel = 0;
double ccc_left_errors[WHEELS_PER_SIDE];
double ccc_right_errors[WHEELS_PER_SIDE];

// PID Controllers
#if CCC_ENABLED
    PID fl_pid(&lw_avg_vels[0], &lw_output[0], &lw_desired_vels[0], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID ml_pid(&lw_avg_vels[1], &lw_output[1], &lw_desired_vels[1], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rl_pid(&lw_avg_vels[2], &lw_output[2], &lw_desired_vels[2], WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID fr_pid(&rw_avg_vels[0], &rw_output[0], &rw_desired_vels[0], WHEEL_KP, WHEEL_KI, WHEEL_KD, REVERSE);
    PID mr_pid(&rw_avg_vels[1], &rw_output[1], &rw_desired_vels[1], WHEEL_KP, WHEEL_KI, WHEEL_KD, REVERSE);
    PID rr_pid(&rw_avg_vels[2], &rw_output[2], &rw_desired_vels[2], WHEEL_KP, WHEEL_KI, WHEEL_KD, REVERSE);

    PID ccc_fl(&ccc_left_errors[0], &lw_desired_vels[0], &left_wheels_desired_vel, CCC_KP, CCC_KI, 0, DIRECT);
    PID ccc_ml(&ccc_left_errors[1], &lw_desired_vels[1], &left_wheels_desired_vel, CCC_KP, CCC_KI, 0, DIRECT);
    PID ccc_rl(&ccc_left_errors[2], &lw_desired_vels[2], &left_wheels_desired_vel, CCC_KP, CCC_KI, 0, DIRECT);
    PID ccc_fr(&ccc_right_errors[0], &rw_desired_vels[0], &right_wheels_desired_vel, CCC_KP, CCC_KI, 0, DIRECT);
    PID ccc_mr(&ccc_right_errors[1], &rw_desired_vels[1], &right_wheels_desired_vel, CCC_KP, CCC_KI, 0, DIRECT);
    PID ccc_rr(&ccc_right_errors[2], &rw_desired_vels[2], &right_wheels_desired_vel, CCC_KP, CCC_KI, 0, DIRECT);
#else
    PID fl_pid(&lw_avg_vels[0], &lw_output[0], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID ml_pid(&lw_avg_vels[1], &lw_output[1], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID rl_pid(&lw_avg_vels[2], &lw_output[2], &left_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, DIRECT);
    PID fr_pid(&rw_avg_vels[0], &rw_output[0], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, REVERSE);
    PID mr_pid(&rw_avg_vels[1], &rw_output[1], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, REVERSE);
    PID rr_pid(&rw_avg_vels[2], &rw_output[2], &right_wheels_desired_vel, WHEEL_KP, WHEEL_KI, WHEEL_KD, REVERSE);
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

void setup_all_pids() {
    #if CCC_ENABLED
        ccc_fl.SetOutputLimits(-1 * ROVER_MAX_SPEED_MPS, ROVER_MAX_SPEED_MPS);
        ccc_ml.SetOutputLimits(-1 * ROVER_MAX_SPEED_MPS, ROVER_MAX_SPEED_MPS);
        ccc_rl.SetOutputLimits(-1 * ROVER_MAX_SPEED_MPS, ROVER_MAX_SPEED_MPS);
        ccc_fr.SetOutputLimits(-1 * ROVER_MAX_SPEED_MPS, ROVER_MAX_SPEED_MPS);
        ccc_mr.SetOutputLimits(-1 * ROVER_MAX_SPEED_MPS, ROVER_MAX_SPEED_MPS);
        ccc_rr.SetOutputLimits(-1 * ROVER_MAX_SPEED_MPS, ROVER_MAX_SPEED_MPS);

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

    // Wheel PID outputs directly to desired duty cycle
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
            set_motor_velocity_targets();

            set_arm_velocities();

            set_servo_positions();

            last_command_time_ms = millis();
        }
        else if (millis() > last_command_time_ms + COMMAND_TIMEOUT_RESET_MS) {
            // If we've not got a valid command recently
            // Set all motor velocities to zero
            set_commands_default();
            set_motor_velocity_targets();
            report_error(NO_COMMANDS_ERROR);
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
            if (millis() - last_encoder_failure < ENCODER_ALLOWABLE_FAILURE_PERIOD_MS) {
                if (++recent_encoder_failures > PID_MAX_ENCODER_FAILURES) {
                    pid_enabled = false;
                }
            }
            else {
                recent_encoder_failures = 0;
            }
            last_encoder_failure = millis();
            report_error(ENCODER_READ_ERROR);
        }
    }

    // If PID is enabled, determine wheel PWM outputs automatically
    if (pid_enabled) {
        #if CCC_ENABLED
        compute_ccc_pids();
        #endif
        compute_wheel_pids();
    }
    // If PID is disabled, then just use open-loop control on the PWM signals
    else {
        set_wheel_pwm_open_loop();
    }
}

void set_wheel_pwm_open_loop() {
    // just convert target velocities straight to pwm outputs, without pid
    float duty_cycle;
    uint8_t wheel_num;

    duty_cycle = 0.5 * (1 + (left_wheels_desired_vel / ROVER_MAX_LIN_SPEED_MPS));
    for (wheel_num = 0; wheel_num < WHEELS_PER_SIDE; wheel_num++) {
        set_pwm_duty_cycle(L_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);
    }

    duty_cycle = 0.5 * (1 + (right_wheels_desired_vel / ROVER_MAX_LIN_SPEED_MPS));
    for (wheel_num = 0; wheel_num < WHEELS_PER_SIDE; wheel_num++) {
        set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[wheel_num], duty_cycle);
    }
}

#if CCC_ENABLED
void compute_ccc_pids() {
    ccc_fl.Compute();
    ccc_ml.Compute();
    ccc_rl.Compute();
    ccc_fr.Compute();
    ccc_mr.Compute();
    ccc_rr.Compute();
}
#endif

void compute_wheel_pids() {
    if (fl_pid.Compute()) { set_pwm_duty_cycle(L_MOTOR_PWM_CHANNELS[0], lw_output[0]); }
    if (ml_pid.Compute()) { set_pwm_duty_cycle(L_MOTOR_PWM_CHANNELS[1], lw_output[1]); }
    if (rl_pid.Compute()) { set_pwm_duty_cycle(L_MOTOR_PWM_CHANNELS[2], lw_output[2]); }
    if (fr_pid.Compute()) { set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[0], rw_output[0]); }
    if (mr_pid.Compute()) { set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[1], rw_output[1]); }
    if (rr_pid.Compute()) { set_pwm_duty_cycle(R_MOTOR_PWM_CHANNELS[2], rw_output[2]); }
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

void set_motor_velocity_targets() {
    float lin_vel = ROVER_MAX_LIN_SPEED_MPS * (commands.lin_vel - 100.0) / 100;
    float ang_vel = ROVER_MAX_ANG_SPEED_RPS * (commands.ang_vel - 100.0) / 100;
    left_wheels_desired_vel  = lin_vel + (ROVER_HALF_WHEEL_SEP_M * ang_vel);
    right_wheels_desired_vel = lin_vel - (ROVER_HALF_WHEEL_SEP_M * ang_vel);
}


void set_servo_positions() {
    // Need to convert from desired position in degrees to required
    // pwm parameters
    uint16_t yaw_tick   = map(commands.yaw_servo_position_deg,
                              0, 180,
                              YAW_SERVO_MIN_TICK, YAW_SERVO_MAX_TICK);
    uint16_t pitch_tick = map(commands.pitch_servo_position_deg,
                              0, 180,
                              PITCH_SERVO_MIN_TICK, PITCH_SERVO_MAX_TICK);

    servo_pwm.setPWM(CAMERA_YAW_PWM, 0, yaw_tick);
    servo_pwm.setPWM(CAMERA_PITCH_PWM, 0, pitch_tick);
}


// TODO I'm sure this can be neatened up
void set_arm_velocities() {
    float duty_cycle, target_speed;
    
    target_speed = -1 * (commands.base_rotation_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_DUTY_CYCLE;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(BASE_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = (commands.arm_actuator_1_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_DUTY_CYCLE;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_1_PWM, duty_cycle);

    target_speed = (commands.arm_actuator_2_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_DUTY_CYCLE;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(ARM_ACTUATOR_2_PWM, duty_cycle);

    target_speed = (commands.wrist_rotation_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_DUTY_CYCLE;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ROTATE_MOTOR_PWM, duty_cycle);

    target_speed = (commands.wrist_actuator_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_DUTY_CYCLE;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(WRIST_ACTUATOR_PWM, duty_cycle);

    target_speed = (commands.gripper_velocity - 100) / 100.0;
    target_speed *= MOTOR_MAX_DUTY_CYCLE;
    duty_cycle = 0.5 * (1 + target_speed);
    set_pwm_duty_cycle(GRIPPER_MOTOR_PWM, duty_cycle);
}

void set_pwm_duty_cycle(uint8_t pwm_num, float duty_cycle) {
    // Do not allow exceeding motor duty cycle limits
    static float min_duty_cycle = (1 - MOTOR_MAX_DUTY_CYCLE) / 2;
    static float max_duty_cycle = 1 - min_duty_cycle;
    duty_cycle = constrain(duty_cycle, min_duty_cycle, max_duty_cycle);
    motor_pwm.setPWM(pwm_num, 0, duty_cycle * (PWM_TICKS - 1));
}

// This whole setup could probably be significantly cleaned up by having a good think
// about what arrays are actually necessary, which need to be held long-term etc. Might
// be easier/more clear to convert everything to metres ASAP.
// Note: This MUST ONLY be called when there is new encoder data available, else it will
// just push out old data and add duplicates of the most recent data
void update_wheel_velocity_estimates() {
    float distance_m, old_vel;
    // For each wheel on either side
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        // Determine how far it travelled over the least recent encoder update interval
        distance_m = lw_encoder_diffs[wheel][ENCODER_HISTORY_LENGTH - 1] * WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV;
        // Determine the velocity for that time interval
        old_vel = 1000 * distance_m / encoder_time_diffs[ENCODER_HISTORY_LENGTH - 1];
        // Subtract that velocity's contribution to the rolling average
        lw_avg_vels[wheel] -= old_vel / ENCODER_HISTORY_LENGTH;

        // Repeat for the right wheel
        distance_m = rw_encoder_diffs[wheel][ENCODER_HISTORY_LENGTH - 1] * WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV;
        old_vel = 1000 * distance_m / encoder_time_diffs[ENCODER_HISTORY_LENGTH - 1] ;
        rw_avg_vels[wheel] -= old_vel / ENCODER_HISTORY_LENGTH;
    }
    
    // Make room for the new encoder time values
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
    
    // New encoder diffs
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        lw_encoder_diffs[wheel][0] = lw_encoder_counts[wheel][0] - lw_encoder_counts[wheel][1];
        rw_encoder_diffs[wheel][0] = rw_encoder_counts[wheel][0] - rw_encoder_counts[wheel][1];
    }
    
    // Final avg vel update
    float new_vel;
    for (uint8_t wheel = 0; wheel < WHEELS_PER_SIDE; wheel++) {
        distance_m = lw_encoder_diffs[wheel][0] * WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV;
        new_vel = 1000 * distance_m / encoder_time_diffs[0];
        lw_avg_vels[wheel] += new_vel / ENCODER_HISTORY_LENGTH;

        distance_m = rw_encoder_diffs[wheel][0] * WHEEL_CIRCUMFERENCE_M / TICKS_PER_WHEEL_REV;
        new_vel = 1000 * distance_m / encoder_time_diffs[0];
        rw_avg_vels[wheel] += new_vel / ENCODER_HISTORY_LENGTH;
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

void set_commands_default() {
    commands.lin_vel = 100;
    commands.ang_vel = 100;
    commands.base_rotation_velocity  = 100;
    commands.arm_actuator_1_velocity = 100;
    commands.arm_actuator_2_velocity = 100;
    commands.wrist_rotation_velocity = 100;
    commands.wrist_actuator_velocity = 100;
    commands.gripper_velocity        = 100;
    commands.yaw_servo_position_deg   = 90;
    commands.pitch_servo_position_deg = 90;
}

