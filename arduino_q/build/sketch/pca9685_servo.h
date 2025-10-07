#line 1 "D:\\GIT\\NO-BUG-QJ\\arduino_q\\pca9685_servo.h"
#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define SG90_MIN_PULSE 150
#define SG90_MAX_PULSE 600
#define SG90_HOME_PULSE 375
#define SG90_RELATIVE_ANGLE -100

extern Adafruit_PWMServoDriver pca;

bool pca9685Init();
uint16_t relativeAngleToPulseSG90(int relativeAngle);
void controlSG90Relative(int channel, int relativeAngle);
void resetSG90(int channel);

#endif
