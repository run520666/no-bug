#line 1 "D:\\GIT\\no-bug\\arduino_test\\servo_test\\servo_test\\slide_servo.cpp"
#include "slide_servo.h"

Servo zx20s_7;
Servo zx20s_8;
Servo zx20s_9;

int pos = 0;
// 七号舵机：初始位置84，左减10，右加10
int zx20s_7_initialAngle = 84;
int zx20s_7_moveAngle = 10;
int zx20s_7_currentAngle = 84;

// 八号舵机：初始位置56，左为140
int zx20s_8_initialAngle = 56;
int zx20s_8_moveAngle = 84;  // 140 - 56 = 84
int zx20s_8_currentAngle = 56;

// 九号舵机：初始位置162，右为56
int zx20s_9_initialAngle = 162;
int zx20s_9_moveAngle = 106;  // 162 - 56 = 106
int zx20s_9_currentAngle = 162;

int direction = 0;

void zx20s_7ChuShiHua() {
  zx20s_7.attach(ZX20S_PIN_7);
  zx20s_7.write(zx20s_7_initialAngle);
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  delay(1000);
  Serial.println("翻转舵机初始化完成");
}

void zx20s_7Left() {
  zx20s_7_currentAngle = zx20s_7_currentAngle - zx20s_7_moveAngle;  // 84 - 10 = 74
  if (zx20s_7_currentAngle < 0) zx20s_7_currentAngle = 0;
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("七号舵机左转到74度");
}

void zx20s_7Right() {
  zx20s_7_currentAngle = zx20s_7_currentAngle + zx20s_7_moveAngle;  // 84 + 10 = 94
  if (zx20s_7_currentAngle > 180) zx20s_7_currentAngle = 180;
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("七号舵机右转到94度");
}

void zx20s_7FuWei() {
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  zx20s_7.write(zx20s_7_currentAngle);
  Serial.println("七号舵机复位到84度");
}

void zx20s_8ChuShiHua() {
  zx20s_8.attach(ZX20S_PIN_8);
  zx20s_8.write(zx20s_8_initialAngle);
  zx20s_8_currentAngle = zx20s_8_initialAngle;
  delay(1000);
  Serial.println("左舵机初始化完成");
}

void zx20s_8Left() {
  int targetAngle = 140;  // 左为140度
  
  // 从当前位置平滑移动到目标位置
  if (zx20s_8_currentAngle < targetAngle) {
    for (pos = zx20s_8_currentAngle; pos <= targetAngle; pos += 1) {
      zx20s_8.write(pos);
      delay(10);
    }
  } else {
    for (pos = zx20s_8_currentAngle; pos >= targetAngle; pos -= 1) {
      zx20s_8.write(pos);
      delay(10);
    }
  }
  
  zx20s_8_currentAngle = targetAngle;
  Serial.println("左滑道已放下到140度");
}

void zx20s_8FuWei() {
  zx20s_8_currentAngle = zx20s_8_initialAngle;
  zx20s_8.write(zx20s_8_currentAngle);
  Serial.println("左舵机复位到56度");
}

void zx20s_9ChuShiHua() {
  zx20s_9.attach(ZX20S_PIN_9);
  zx20s_9.write(zx20s_9_initialAngle);
  zx20s_9_currentAngle = zx20s_9_initialAngle;
  delay(1000);
  Serial.println("右舵机初始化完成");
}

void zx20s_9Right() {
  int targetAngle = 56;  // 右为56度
  
  // 从当前位置平滑移动到目标位置
  if (zx20s_9_currentAngle < targetAngle) {
    for (pos = zx20s_9_currentAngle; pos <= targetAngle; pos += 1) {
      zx20s_9.write(pos);
      delay(15);
    }
  } else {
    for (pos = zx20s_9_currentAngle; pos >= targetAngle; pos -= 1) {
      zx20s_9.write(pos);
      delay(15);
    }
  }
  
  zx20s_9_currentAngle = targetAngle;
  Serial.println("右滑道已放下到56度");
}

void zx20s_9FuWei() {
  zx20s_9_currentAngle = zx20s_9_initialAngle;
  zx20s_9.write(zx20s_9_currentAngle);
  Serial.println("右舵机复位到162度");
}