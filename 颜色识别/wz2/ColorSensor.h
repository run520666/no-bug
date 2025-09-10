#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Wire.h>
#include "Common.h"

// 颜色传感器地址（默认）
#define COLOR_SENSOR_ADDR 0x4f

class ColorSensor {
public:
  ColorSensor();
  ~ColorSensor() {};
  
  // 初始化（选择TCA通道）
  bool init(uint8_t tcaChannel);
  // 读取RGB数据
  bool readRGB(int rgb[3]);
  // 读取HSL数据
  bool readHSL(int hsl[3]);
  // 根据HSL识别颜色
  BallColor identifyColor(int hsl[3], int rgb[3]);
  // 切换TCA通道
  void selectTCAChannel(uint8_t channel);

private:
  uint8_t _tcaChannel;
  // 暗光补偿
  const int _darkCompensation = 50;
  // Ping传感器（同步初始化）
  bool pingSensor();
};

#endif