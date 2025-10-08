#line 1 "D:\\GIT\\NO-BUG-QJ\\arduino_q\\slide_servo.cpp"
#include "slide_servo.h"

Servo zx20s_7;
Servo zx20s_8;
Servo zx20s_9;

int pos = 0;
int zx20s_7_initialAngle = 90;
int zx20s_7_moveAngle = 20;
int zx20s_7_currentAngle = 90;
int zx20s_8_initialAngle = 165;
int zx20s_8_moveAngle = 95;
int zx20s_8_currentAngle = 165;
int zx20s_9_initialAngle = 35;
int zx20s_9_moveAngle = 115;
int zx20s_9_currentAngle = 35;
int direction = 0;

void zx20s_7ChuShiHua() {
  zx20s_7.attach(ZX20S_PIN_7);
  zx20s_7.write(zx20s_7_initialAngle);
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  delay(1000);
  Serial.println("翻转舵机初始化完成");
}

void zx20s_7Left() {
  zx20s_7_currentAngle = zx20s_7_currentAngle + zx20s_7_moveAngle;
  if (zx20s_7_currentAngle > 180) zx20s_7_currentAngle = 180;
  zx20s_7.write(zx20s_7_currentAngle);
}

void zx20s_7Right() {
  zx20s_7_currentAngle = zx20s_7_currentAngle - zx20s_7_moveAngle;
  if (zx20s_7_currentAngle < 0) zx20s_7_currentAngle = 0;
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("右翻 ");
}

void zx20s_7FuWei() {
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("复位初始角度");
}

void zx20s_8ChuShiHua() {
  zx20s_8.attach(ZX20S_PIN_8);
  zx20s_8.write(zx20s_8_initialAngle);
  zx20s_8_currentAngle = zx20s_8_initialAngle;
  delay(1000);
  Serial.println("左舵机初始化完成");
}

void zx20s_8Left() {
  if (zx20s_8_currentAngle < 0) zx20s_8_currentAngle = 0;
  zx20s_8_currentAngle = zx20s_8_currentAngle - zx20s_8_moveAngle;
   
  zx20s_8.write(zx20s_8_currentAngle);
  for (pos = 180; pos >= zx20s_8_currentAngle; pos -= 1) {
    zx20s_8.write(pos);
    delay(10);
  }
  Serial.println("左滑道已放下");
}

void zx20s_8FuWei() {
  zx20s_8_currentAngle = zx20s_8_initialAngle;
  zx20s_8.write(zx20s_8_currentAngle);
  Serial.println("左舵机复位初始角度");
}

void zx20s_9ChuShiHua() {
  zx20s_9.attach(ZX20S_PIN_9);
  zx20s_9.write(zx20s_9_initialAngle);
  zx20s_9_currentAngle = zx20s_9_initialAngle;
  delay(1000);
  Serial.println("右舵机初始化完成");
}

void zx20s_9Right() {
  zx20s_9_currentAngle = zx20s_9_currentAngle + zx20s_9_moveAngle;
  if (zx20s_9_currentAngle > 180) zx20s_9_currentAngle = 180;
  zx20s_9.write(zx20s_9_currentAngle);
  for (pos = 0; pos <= zx20s_9_currentAngle; pos += 1) {
    zx20s_9.write(pos);
    delay(15);
  }
  Serial.println("右滑道已放下");
}

void zx20s_9FuWei() {
  zx20s_9_currentAngle = zx20s_9_initialAngle;
  zx20s_9.write(zx20s_9_currentAngle);
  Serial.println("右舵机复位初始角度");
}
