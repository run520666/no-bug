#ifndef PCA9685_H
#define PCA9685_H

#include <Wire.h>
#include "Common.h"

// PCA9685默认地址
#define PCA9685_ADDR 0x40
// 寄存器定义
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define PCA9685_LED0_ON_L 0x06

class PCA9685 {
public:
  PCA9685();
  ~PCA9685() {};
  
  // 初始化（选择TCA通道）
  bool init(uint8_t tcaChannel);
  // 设置PWM频率
  void setPWMFreq(float freq);
  // 切换TCA通道
  void selectTCAChannel(uint8_t channel);
  // 控制舵机角度（0-180度）
  void setServoAngle(uint8_t channel, uint8_t angle);

private:
  uint8_t _tcaChannel;
  // 写入PCA9685寄存器
  void writeReg(uint8_t reg, uint8_t value);
  // 读取PCA9685寄存器
  uint8_t readReg(uint8_t reg);
  // 计算舵机PWM值（角度->PWM）
  uint16_t angleToPWM(uint8_t angle);
};

#endif