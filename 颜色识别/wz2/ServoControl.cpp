#include "ServoControl.h"
#include "arduino.h"

ServoControl::ServoControl() {
  _pca = nullptr;
  _startAngle = 0; // 起始角度默认为0度（上电位置）
}

// 初始化所有舵机
bool ServoControl::init(PCA9685 &pca) {
  _pca = &pca;
  
  // 初始化ZX20S舵机
  if (!_mainZX20S.attach(ZX20S_MAIN_PIN)) return false;
  if (!_d8ZX20S.attach(ZX20S_D8_PIN)) return false;
  if (!_d10ZX20S.attach(ZX20S_D10_PIN)) return false;
  
  // 初始化PCA9685舵机（50Hz）
  _pca->setPWMFreq(50);
  
  // 回到起始位置
  setMainZX20SAngle(_startAngle);
  setD8ZX20SAngle(_startAngle);
  setD10ZX20SAngle(_startAngle);
  delay(500);
  
  return true;
}

// 控制主ZX20S舵机角度
void ServoControl::setMainZX20SAngle(uint8_t angle) {
  angle = constrain(angle, 0, 180);
  _mainZX20S.write(angle);
  delay(200); // 等待舵机到位
}

// 控制D8口ZX20S舵机
void ServoControl::setD8ZX20SAngle(uint8_t angle) {
  angle = constrain(angle, 0, 180);
  _d8ZX20S.write(angle);
  delay(200);
}

// 控制D10口ZX20S舵机
void ServoControl::setD10ZX20SAngle(uint8_t angle) {
  angle = constrain(angle, 0, 180);
  _d10ZX20S.write(angle);
  delay(200);
}

// 根据颜色查找PCA通道（colorAngleMap：索引0-6对应角度0-144度的颜色）
int ServoControl::findPCAChannelByColor(BallColor color, int colorAngleMap[7]) {
  for (int i = 0; i < 7; i++) {
    if (colorAngleMap[i] == color) {
      return ANGLE_TO_PCA_CHANNEL[i];
    }
  }
  return -1; // 未找到
}

// 释放指定颜色的小球
void ServoControl::releaseBall(BallColor color, int colorAngleMap[7]) {
  if (_pca == nullptr) return;
  
  // 特殊处理：R对应释放SG90（100度）
  if (color == COLOR_RED) {
    for (int i = 0; i < 7; i++) {
      _pca->setServoAngle(ANGLE_TO_PCA_CHANNEL[i], SG90_RELEASE_ANGLE);
      delay(100);
    }
    return;
  }
  
  // 其他颜色：找到对应通道并释放
  int channel = findPCAChannelByColor(color, colorAngleMap);
  if (channel != -1) {
    _pca->setServoAngle(channel, SG90_RELEASE_ANGLE);
    delay(500); // 等待释放完成
  }
}

// 释放后联动动作：D8逆时针100度 -> D10逆时针100度
void ServoControl::releaseLinkAction() {
  // 逆时针100度（相对于起始位置）
  setD8ZX20SAngle(_startAngle + ZX20S_RELEASE_ANGLE);
  delay(500);
  setD10ZX20SAngle(_startAngle + ZX20S_RELEASE_ANGLE);
  
  // 可根据需求添加复位逻辑
  // delay(1000);
  // setD8ZX20SAngle(_startAngle);
  // setD10ZX20SAngle(_startAngle);
}