#line 1 "D:\\GIT\\no-bug\\arduino_test\\servo_test\\servo_test\\servo_test.ino"
#include <Arduino.h>
#include <Servo.h>

Servo myServo;

void setup() {
  myServo.attach(9);
}

void loop() {
  myServo.write(90);
  delay(1000);
  myServo.write(0);
  delay(1000);
}
}

