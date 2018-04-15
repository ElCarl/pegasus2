#include <Servo.h>

Servo servo1;
Servo servo2;
uint8_t angle_rate = 0;
uint8_t angle = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  servo1.attach(9);
  servo2.attach(10);
  servo1.write(-90);
  servo2.write(90);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available() > 0) {
    angle_rate = Serial.read();
    angle += angle_rate;
    if (angle < 0){
      servo1.write(-90 - angle);
      servo2.write(90);
    }
    else if (angle >= 0){
      servo1.write(-90);
      servo2.write(90 - angle);
    }
  }
}
