#ifndef PCA9685_SERVO_H
#define PCA9685_SERVO_H

#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define SG90_MIN_PULSE 150
#define SG90_MAX_PULSE 600
#define SG90_HOME_PULSE 375
#define SG90_RELATIVE_ANGLE -95

extern Adafruit_PWMServoDriver pca;

// 初始化PCA9685
bool pca9685Init();

// 设置初始角度（仅记录，不移动舵机）
void setSG90InitialAngle(int channel, int angle);

// 设置绝对角度（直接移动到指定角度）
void setSG90AbsoluteAngle(int channel, int angle);

// 相对角度控制（基于当前位置移动）
void controlSG90Relative(int channel, int relativeAngle);

// 获取当前角度
int getSG90CurrentAngle(int channel);

// 复位到90度
void resetSG90(int channel);

#endif