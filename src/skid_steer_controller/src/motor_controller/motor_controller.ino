#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Electrical & physical constants
#define MOTOR_MAX_SPEED 0.7  // Normalised - so 0.7 is 70% duty cycle rather than 0.7 m/s
#define MOTOR_1_RELATIVE_SPEED 1  // Some motors may be slower than others:
#define MOTOR_2_RELATIVE_SPEED 1  // this should offer a way of addressing this
#define MOTOR_3_RELATIVE_SPEED 1
#define MOTOR_4_RELATIVE_SPEED 1
#define MOTOR_5_RELATIVE_SPEED 1
#define MOTOR_6_RELATIVE_SPEED 1

// Geometric constants
#define WHEELBASE_WIDTH_M 0.7    // metres, approximate
#define WHEELBASE_LENGTH_M 0.908 // metres
#define WHEEL_RADIUS_M 0.2       // metres, approximate

// PWM pin constants
#define L_FRONT_MOTOR_PWM 0
#define L_MID_MOTOR_PWM 1
#define L_REAR_MOTOR_PWM 2
#define R_FRONT_MOTOR_PWM 3
#define R_MID_MOTOR_PWM 4
#define R_REAR_MOTOR_PWM 5

// Serial constants
#define BEGIN_MESSAGE_BYTE 252
#define ARM_MESSAGE_BYTE 253
#define DRIVE_MESSAGE_BYTE 254
#define END_MESSAGE_BYTE 255

// Other IO constants
#define OUTPUT_ENABLE_PIN 4  // Pin chosen at random, change as appropriate
#define PWM_CHANNELS 16
#define PWM_MAX_TICK 4096

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
    Serial.begin(38400);

    pinMode(LED_BUILTIN, OUTPUT);  // Initialise the Arduino built in LED to allow it to indicate status

    pwm.begin();
    pwm.setPWMFreq(1600);  // Max pwm freq for motor board is 20k, max for pwm board is 1600

    Wire.setClock(400000);  // Some mode that speeds something up - if something weird breaks this should be the first thign to go

    // Start PWM with all motors stationary
    init_pwm();

    // Wait for handshake from Python script sending velocity commands
    // PLACEHOLDER
}

void loop() {
    byte message_type;

    // Wait until the start of a message is found - this ensures that we don't misinterpret a message
    while (Serial.read() != BEGIN_MESSAGE_BYTE) {}

    message_type = Serial.read();

    if (message_type == ARM_MESSAGE_BYTE) {
        // Not currently implemented
        // Should probably stop the drive motors when messing with the arm
        // Random thought for me to come across later: maybe implement the d-pad as a
        // rough control for the drive (slowed way down e.g. [0,0.1])
    }
    else if (message_type == DRIVE_MESSAGE_BYTE) {
        byte linear_velocity, angular_velocity;
        linear_velocity = Serial.read();   // Both of these values should be in range [0,200]
        angular_velocity = Serial.read();  // and must be converted to [-100,100]

        float rover_target_velocity[2];
        rover_target_velocity[0] = (linear_velocity - 100) / 100.0;
        rover_target_velocity[1] = (angular_velocity - 100) / 100.0;

        float wheel_speeds[2];
        get_control_outputs(wheel_speeds, rover_target_velocity);

        set_wheel_speeds(wheel_speeds);
    }
    while (Serial.read() != END_MESSAGE_BYTE) {}
}

void init_pwm() {
    for (uint8_t pwm_num = 0; pwm_num < PWM_CHANNELS; pwm_num++) {
        pwm.setPWM(pwm_num, 0, PWM_MAX_TICK / 2);  // Sets PWM to 50% which should stop motors
    }
}

/* void set_pwm_duty_cycle(uint8_t pwm_num, float duty_cycle) {
 *     pwm.setPWM(pwm_num, 0, (int)(duty_cycle * PWM_MAX_TICK));
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

void set_wheel_speeds(wheel_speeds) {
    // wheel_speeds is a float array: {left_wheels_relative_speed, right_wheels_relative_speed}
    
}
