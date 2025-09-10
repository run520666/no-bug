#ifndef SERVOCONTROL_H
#define SERVOCONTROL_H

#include <Servo.h>
#include "Common.h"
#include "PCA9685.h"

class ServoControl {
public:
  ServoControl();
  ~ServoControl() {};
  
  // 初始化所有舵机
  bool init(PCA9685 &pca);
  // 控制主ZX20S舵机转动到指定角度
  void setMainZX20SAngle(uint8_t angle);
  // 控制D8口ZX20S舵机
  void setD8ZX20SAngle(uint8_t angle);
  // 控制D10口ZX20S舵机
  void setD10ZX20SAngle(uint8_t angle);
  // 释放指定颜色的小球（根据颜色找到PCA通道）
  void releaseBall(BallColor color, int colorAngleMap[7]);
  // 释放后联动D8和D10舵机
  void releaseLinkAction();

private:
  Servo _mainZX20S;    // 主ZX20S舵机（D9）
  Servo _d8ZX20S;      // D8口ZX20S舵机
  Servo _d10ZX20S;     // D10口ZX20S舵机
  PCA9685 *_pca;       // PCA9685指针
  uint8_t _startAngle;  // 舵机起始角度（上电位置）
  
  // 根据颜色查找对应的PCA通道
  int findPCAChannelByColor(BallColor color, int colorAngleMap[7]);
};

#endif