#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>
#include <SoftEasyTransfer.h>

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
const unsigned int  PWM_FREQUENCY      = 1600;    // Max supported freq of current PWM board is 1600 Hz
const unsigned long PWM_I2C_CLOCKSPEED = 400000;  // I2C "fast" mode @ 400 kHz
const uint8_t L_FRONT_MOTOR_PWM = 0;
const uint8_t L_MID_MOTOR_PWM   = 1;
const uint8_t L_REAR_MOTOR_PWM  = 2;
const uint8_t R_FRONT_MOTOR_PWM = 3;
const uint8_t R_MID_MOTOR_PWM   = 4;
const uint8_t R_REAR_MOTOR_PWM  = 5;
const uint8_t L_MOTOR_PWM_CHANNELS[] = {L_FRONT_MOTOR_PWM, L_MID_MOTOR_PWM, L_REAR_MOTOR_PWM};
const uint8_t R_MOTOR_PWM_CHANNELS[] = {R_FRONT_MOTOR_PWM, R_MID_MOTOR_PWM, R_REAR_MOTOR_PWM};

// Serial constants
const unsigned long HS_BAUDRATE = 38400;  // For comms with the Braswell chip (arduino_communicator_node)
const unsigned long SS_BAUDRATE = 38400;  // For comms with encoder counter Uno
const uint8_t SOFTSERIAL_RX_PIN = 2;
const uint8_t SOFTSERIAL_TX_PIN = 3;
const unsigned long TIMEOUT_MS  = 5000;   // TODO - actually implement this!
const byte ENCODER_DATA_BYTE    = 250;
const byte SERIAL_READY_BYTE    = 251;
const byte BEGIN_MESSAGE_BYTE   = 252;
const byte ARM_MESSAGE_BYTE     = 253;
const byte DRIVE_MESSAGE_BYTE   = 254;
const byte END_MESSAGE_BYTE     = 255;

// Other IO constants
const uint8_t MOTOR_ENABLE_PIN = 4;  // Pin chosen at random, change as appropriate
const uint8_t PWM_CHANNELS = 16;
const unsigned int PWM_TICKS = 4096;

// Global variables
uint32_t enc_count_handshake_time_ms;
uint32_t handshake_offset_ms;

// Initialise the PWM board object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Initialise the software serial
SoftwareSerial soft_serial(SOFTSERIAL_RX_PIN, SOFTSERIAL_TX_PIN);

// Initialise the SoftEasyTransfer object
SoftEasyTransfer EasyRX;

// Define the SoftEasyTransfer RX data structure
struct RECEIVE_DATA_STRUCTURE {
    uint32_t tick_stamp_ms;  // Timestamp of encoder counts in ms since encoder counter arduino started
    int32_t encoder_counts[N_ENCS];
};

RECEIVE_DATA_STRUCTURE encoder_counts_struct;

void setup() {
    // Pin modes
    pinMode(LED_BUILTIN, OUTPUT);  // Initialise the Arduino built in LED to allow it to indicate status
    
    // Begin serial connections
    Serial.begin(HS_BAUDRATE);
    soft_serial.begin(SS_BAUDRATE);

    // Begin PWM
    pwm.begin();
    pwm.setPWMFreq(PWM_FREQUENCY);
    Wire.setClock(PWM_I2C_CLOCKSPEED);  // Sets the I2C bus speed

    // Start PWM with all motors stationary
    init_pwm();
    
    // Begin SoftEasyTransfer
    EasyRX.begin(details(encoder_counts_struct), &soft_serial);
    
    // Send the serial ready byte to indicate readiness for data while awaiting readiness confirmation from encoder counter Uno
    while (soft_serial.read() != SERIAL_READY_BYTE) {
        soft_serial.write(SERIAL_READY_BYTE);
        delay(100);
        // May need to implement some way to calibrate for different start times of Arduino boards? Offset in ms between boards?
    }
    
    enc_count_handshake_time_ms = millis();
    
    // Send the serial ready byte to indicate readiness for data while awaiting readiness confirmation from Braswell chip
    while (Serial.read() != SERIAL_READY_BYTE) {
        Serial.write(SERIAL_READY_BYTE);
        delay(100);
    }
    
    handshake_offset_ms = millis() - enc_count_handshake_time_ms;
}

void loop() {
    // Check if any movement commands have been sent
    if (Serial.available() > 0) {
        byte message_type;

        if (DEBUG_MODE) { Serial.print("Awaiting BEGIN_MESSAGE_BYTE... "); }
        // Wait until the start of a message is found - this ensures that we don't misinterpret a message
        while (Serial.read() != BEGIN_MESSAGE_BYTE) {}

        if (DEBUG_MODE) { Serial.println("BEGIN_MESSAGE_BYTE received"); }

        if (DEBUG_MODE) { Serial.print("Awaiting message_type byte... "); }
        while (Serial.available() == 0) {}
        message_type = Serial.read();
        if (DEBUG_MODE) { Serial.print("Message type: "); Serial.println(message_type); }
        
        // Could rejig this a bit. Instead of worrying about message type on this end, just send all movement commands to this sketch (e.g. linvel, angvel, baserot, wristrot etc.). Simplifies transmission a little, and offloads complexity to the arduino_communicator_node which is easier to debug. Can then more easily send e.g. slow movement commmands while manipulating the arm

        if (message_type == ARM_MESSAGE_BYTE) {
            if (DEBUG_MODE) { Serial.println("ARM_MESSAGE_BYTE received"); }
            // Not currently implemented
            // Should probably stop the drive motors when messing with the arm
            // Random thought for me to come across later: maybe implement the d-pad as a
            // rough control for the drive (slowed way down e.g. [0,0.1])
        }
        else if (message_type == DRIVE_MESSAGE_BYTE) {
            if (DEBUG_MODE) { Serial.println("DRIVE_MESSAGE_BYTE received"); }
            byte linear_velocity, angular_velocity;
            while (Serial.available() < 2) {}
            linear_velocity = Serial.read();   // Both of these values should be in range [0,200]
            angular_velocity = Serial.read();  // and must be converted to [-1,1]

            // If the velocities are both in the accepted range
            if (linear_velocity <= 200 && angular_velocity <= 200) {
                // Calculate & set the wheel speeds
                float rover_target_velocity[2];
                rover_target_velocity[0] = (linear_velocity - 100) / 100.0;
                rover_target_velocity[1] = (angular_velocity - 100) / 100.0;

                float wheel_speeds[2];
                get_control_outputs(wheel_speeds, rover_target_velocity);

                set_wheel_speeds(wheel_speeds);
            }
            //  If either of the speeds is wrong, we've probably received a command byte by
            //  mistake - so disregard the message.
        }

        if (DEBUG_MODE) { Serial.println("Awaiting END_MESSAGE_BYTE"); }
        while (Serial.read() != END_MESSAGE_BYTE) {}
        if (DEBUG_MODE) { Serial.println("END_MESSAGE_BYTE received"); }
    }
    
    // If we've successfully received data from the encoder counter Uno
    if (EasyRX.receiveData()) {
        // Convert timestamp to this processor's reference timeframe
        encoder_counts_struct.tick_stamp_ms -= handshake_offset_ms;
        // Transmit encoder counts to the Braswell chip
        send_encoder_data();
        // Probably will also do e.g. speed estimation calcs here? If implementing PID control of wheels
    }
}

void init_pwm() {
    for (uint8_t pwm_num = 0; pwm_num < PWM_CHANNELS; pwm_num++) {
        pwm.setPWM(pwm_num, 0, PWM_TICKS / 2);  // Sets PWM to 50% which should stop motors
    }
}

/* void set_pwm_duty_cycle(uint8_t pwm_num, float duty_cycle) {
 *     pwm.setPWM(pwm_num, 0, (int)(duty_cycle * PWM_TICKS));
 * }
 */

// Given the desired (normalised) rover target velocity, determine the required (normalised) velocity for each set of wheels
// TODO: maybe this should be using integers not floats? Wait to see if speed is a problem first
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
        pwm.setPWM(L_MOTOR_PWM_CHANNELS[wheel_num], 0, (PWM_TICKS - 1) * duty_cycle);

        // Then right wheel - negative since they turn opposite to the left wheels
        target_wheel_speed = -1 * wheel_speeds[1] * R_MOTOR_RELATIVE_SPEEDS[wheel_num] * MOTOR_MAX_SPEED;
        duty_cycle = 0.5 * (1 + target_wheel_speed);
        pwm.setPWM(R_MOTOR_PWM_CHANNELS[wheel_num], 0, (PWM_TICKS - 1) * duty_cycle);
    }
}

void halt_motors() {
    // This should disable the pin connected to the PWM input of each motor driver.
    // Since we are using locked antiphase PWM drive for the motors, this will prevent
    // any current reaching the motors.
    digitalWrite(MOTOR_ENABLE_PIN, LOW);
}

void start_motors() {
    // Opposite of halt_motors!
    digitalWrite(MOTOR_ENABLE_PIN, HIGH);
}

void send_encoder_data() {
    Serial.write(ENCODER_DATA_BYTE);
    byte buffer[4];
    uint8_t checksum = 0;
    
    // Write the timestamp to serial, LSB to MSB
    long_to_bytes(encoder_counts_struct.tick_stamp_ms, buffer);
    for (uint8_t b = 0; b < 4; b++) {
        Serial.write(buffer[b]);
        checksum ^= buffer[b];
    }
    // Then each of the encoders, same order
    for (uint8_t encoder = 0; encoder < N_ENCS; encoder++) {
        long_to_bytes(encoder_counts_struct.encoder_counts[encoder], buffer);
        for (uint8_t b = 0; b < 4; b++) {
            Serial.write(buffer[b]);
            checksum ^= buffer[b];
        }
    }
    // Then the checksum
    long_to_bytes(checksum, buffer);
    for (uint8_t b = 0; b < 4; b++) {
        Serial.write(buffer[b]);
    }
    Serial.write(END_MESSAGE_BYTE);
}

// Converts a long to an array of 4 bytes for serial transmission
void long_to_bytes(int32_t val, byte bytes[]) {
    bytes[0] =  val        & 255;
    bytes[1] = (val >> 8)  & 255;
    bytes[2] = (val >> 16) & 255;
    bytes[3] = (val >> 24) & 255;
}
