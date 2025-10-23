#include "slide_servo.h"

// ========== 舵机角度配置区 - 修改这里即可调整各舵机位置 ==========
// 七号舵机 (翻转舵机) 角度定义
int zx20s_7_initialAngle = 84;  // 七号初始位置
int zx20s_7_leftAngle = 75;     // 七号左位置
int zx20s_7_rightAngle = 96;    // 七号右位置

// 八号舵机 (左滑道) 角度定义
int zx20s_8_initialAngle = 56;  // 八号初始位置
int zx20s_8_leftAngle = 140;    // 八号左位置

// 九号舵机 (右滑道) 角度定义
int zx20s_9_initialAngle = 159; // 九号初始位置
int zx20s_9_rightAngle = 59;    // 九号右位置
// ==============================================================

Servo zx20s_7;
Servo zx20s_8;
Servo zx20s_9;

// 记录当前角度用于平滑移动
int zx20s_7_currentAngle = 84;
int zx20s_8_currentAngle = 56;
int zx20s_9_currentAngle = 159;

// 平滑移动延时(毫秒),可调节移动速度
int servoMoveDelay = 7;

// 平滑移动函数
void smoothMove(Servo &servo, int &currentAngle, int targetAngle) {
  if (currentAngle < targetAngle) {
    for (int pos = currentAngle; pos <= targetAngle; pos++) {
      servo.write(pos);
      delay(servoMoveDelay);
    }
  } else {
    for (int pos = currentAngle; pos >= targetAngle; pos--) {
      servo.write(pos);
      delay(servoMoveDelay);
    }
  }
  currentAngle = targetAngle;
}

void zx20s_7ChuShiHua() {
  zx20s_7.attach(ZX20S_PIN_7);
  zx20s_7.write(zx20s_7_initialAngle);
  zx20s_7_currentAngle = zx20s_7_initialAngle;
  delay(1000);
  Serial.println("翻转舵机初始化完成");
}

void zx20s_7Left() {
  smoothMove(zx20s_7, zx20s_7_currentAngle, zx20s_7_leftAngle);
  Serial.println("左翻");
}

void zx20s_7Right() {
  smoothMove(zx20s_7, zx20s_7_currentAngle, zx20s_7_rightAngle);
  Serial.println("右翻");
}

void zx20s_7FuWei() {
  smoothMove(zx20s_7, zx20s_7_currentAngle, zx20s_7_initialAngle);
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
  smoothMove(zx20s_8, zx20s_8_currentAngle, zx20s_8_leftAngle);
  Serial.println("左滑道已放下");
}

void zx20s_8FuWei() {
  smoothMove(zx20s_8, zx20s_8_currentAngle, zx20s_8_initialAngle);
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
  smoothMove(zx20s_9, zx20s_9_currentAngle, zx20s_9_rightAngle);
  Serial.println("右滑道已放下");
}

void zx20s_9FuWei() {
  smoothMove(zx20s_9, zx20s_9_currentAngle, zx20s_9_initialAngle);
  Serial.println("右舵机复位初始角度");
}
