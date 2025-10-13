#line 1 "D:\\GIT\\no-bug\\arduino_q\\slide_servo.h"
#ifndef SLIDE_SERVO_H
#define SLIDE_SERVO_H

#include <Arduino.h>
#include <Servo.h>

#define ZX20S_PIN_7 7
#define ZX20S_PIN_8 8
#define ZX20S_PIN_9 9

extern Servo zx20s_7;
extern Servo zx20s_8;
extern Servo zx20s_9;

extern int pos;
extern int zx20s_7_initialAngle;
extern int zx20s_7_moveAngle;
extern int zx20s_7_currentAngle;
extern int zx20s_8_initialAngle;
extern int zx20s_8_moveAngle;
extern int zx20s_8_currentAngle;
extern int zx20s_9_initialAngle;
extern int zx20s_9_moveAngle;
extern int zx20s_9_currentAngle;
extern int direction;

void zx20s_7ChuShiHua();
void zx20s_7Left();
void zx20s_7Right();
void zx20s_7FuWei();
void zx20s_8ChuShiHua();
void zx20s_8Left();
void zx20s_8FuWei();
void zx20s_9ChuShiHua();
void zx20s_9Right();
void zx20s_9FuWei();

#endif
